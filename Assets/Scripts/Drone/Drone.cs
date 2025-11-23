using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;
using Accord.Math;
using Vector3 = UnityEngine.Vector3;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(PlayerInput))]
public class Drone : MonoBehaviour
{
    [Header("Modifier Presets")]
    [Tooltip("The preset for estimating the drone's tilt and angular rates.")]
    [SerializeField] private TiltEstimator tiltEstimatorPreset;
    [Tooltip("The preset for calculating desired roll and pitch. Assign a TargetModifier asset.")]
    [SerializeField] private TargetModifier targetModifierPreset;
    [Tooltip("The preset for calculating rotor forces. Assign a RotorForceCalculator asset.")]
    [SerializeField] private RotorForceCalculator rotorForceCalculatorPreset;
    [Tooltip("Optional: The preset for smoothing rotor forces. Assign a FluentRotorSmoother asset.")]
    [SerializeField] private FluentRotorSmoother fluentRotorPreset;
    [Tooltip("Optional: The preset for ground effect. Assign a GroundEffect asset.")]
    [SerializeField] private GroundEffect groundEffectPreset;
    [Tooltip("Optional: The preset for velocity thrust influence. Assign a VelocityThrustInfluence asset.")]
    [SerializeField] private VelocityThrustInfluence velocityThrustInfluencePreset;

    [Header("Core Physics")]
    [Tooltip("Proportional gain for correcting roll and pitch errors. Higher values lead to faster, more aggressive corrections.")]
    [SerializeField] public float tiltGain = 5f;
    [Tooltip("Damping factor for roll and pitch corrections to prevent overshooting and oscillations.")]
    [SerializeField] public float tiltDamping = 1f;
    [Space]
    [Tooltip("The maximum yaw rotation speed in degrees per second.")]
    [SerializeField] public float maxYawRate = 2f;
    [Tooltip("Proportional gain for correcting yaw errors.")]
    [SerializeField] public float yawGain = 2f;
    [Space]
    [Tooltip("The maximum vertical speed in meters per second.")]
    [SerializeField] public float maxVerticalVelocity = 5f;
    [Tooltip("Proportional gain for correcting vertical velocity errors.")]
    [SerializeField] public float verticalGain = 1f;
    [Space]
    [Tooltip("A multiplier for the overall thrust of the drone.")]
    [SerializeField] public float thrustMultiplier = 1f;
    [Tooltip("A coefficient representing the torque produced by rotor drag, used for yaw control.")]
    [SerializeField] public float dragMultiplier = 0.01f;
    [Tooltip("A multiplier for the yaw control torque.")]
    [SerializeField] public float yawTorqueMultiplier = 0.1f;
    [Tooltip("A coefficient for simulating air resistance (drag) on the drone's body.")]
    [SerializeField] public float airResistanceCoefficient = 0.02f;
    [Space]
    [Tooltip("The distance of each rotor from the center of the drone.")]
    [SerializeField] public float rotorDistance = 0.5f;
    [Tooltip("The layout of the rotors. True for '+' configuration, False for 'x' configuration.")]
    [SerializeField] public bool isRotorInFront = true;

    [Space]
    [Header("Events")]
    public UnityEvent OnSwitchCamera;
    public UnityEvent OnRestart;
    public UnityEvent OnToggleUI;

    public Rigidbody Rb { get; private set; }
    public Vector2 LeftJoystick { get; private set; }
    public Vector2 RightJoystick { get; private set; }

    public Vector3 LocalLinearVelocity { get; private set; }
    public float CurrentRoll { get; private set; }
    public float CurrentPitch { get; private set; }
    public float RollRate { get; private set; }
    public float PitchRate { get; private set; }
    public float CurrentYawVelocity { get; private set; }
    public float CurrentVerticalVelocity { get; private set; }
    public float Mass { get; private set; }

    public Vector3[] RotorPoses { get; private set; }
    public double[,] ControlToRotorMatrix { get; private set; }

    [HideInInspector] public float[] RotorForcesArray = new float[4];

    public TiltEstimator CurrentTiltEstimator { get; private set; }
    public TargetModifier CurrentTargetModifier { get; private set; }
    public RotorForceCalculator CurrentRotorForceCalculator { get; private set; }
    public FluentRotorSmoother CurrentFluentRotor { get; private set; }
    public GroundEffect CurrentGroundEffect { get; private set; }
    public VelocityThrustInfluence CurrentVelocityThrustInfluence { get; private set; }

    void Awake()
    {
        Rb = GetComponent<Rigidbody>();

        InitializeModifiers();
    }

    private void InitializeModifiers()
    {
        if (tiltEstimatorPreset != null)
        {
            CurrentTiltEstimator = Instantiate(tiltEstimatorPreset);
            CurrentTiltEstimator.Initialize(this);
        }
        if (targetModifierPreset != null) CurrentTargetModifier = Instantiate(targetModifierPreset);
        if (rotorForceCalculatorPreset != null) CurrentRotorForceCalculator = Instantiate(rotorForceCalculatorPreset);
        if (fluentRotorPreset != null) CurrentFluentRotor = Instantiate(fluentRotorPreset);
        if (groundEffectPreset != null) CurrentGroundEffect = Instantiate(groundEffectPreset);
        if (velocityThrustInfluencePreset != null) CurrentVelocityThrustInfluence = Instantiate(velocityThrustInfluencePreset);
    }

    void FixedUpdate()
    {
        if (CurrentTiltEstimator == null)
        {
            Debug.LogError("No TiltEstimator assigned to the drone!");
            return;
        }
        CurrentTiltEstimator.Estimate(this);

        UpdateCurrentState();

        RunControlPipeline();
    }

    private void UpdateCurrentState()
    {
        Mass = Rb.mass;
        CurrentRoll = -CurrentTiltEstimator.RollAngle * Mathf.Deg2Rad;
        CurrentPitch = CurrentTiltEstimator.PitchAngle * Mathf.Deg2Rad;
        RollRate = -CurrentTiltEstimator.RollRate * Mathf.Deg2Rad;
        PitchRate = CurrentTiltEstimator.PitchRate * Mathf.Deg2Rad;

        LocalLinearVelocity = Quaternion.Inverse(Quaternion.Euler(0, transform.eulerAngles.y, 0)) * Rb.linearVelocity;
        CurrentYawVelocity = transform.InverseTransformDirection(Rb.angularVelocity).y * Mathf.Deg2Rad;
        CurrentVerticalVelocity = Rb.linearVelocity.y;

        RotorPoses = Enumerable.Range(0, 4).Select(i => {
            float placementDegree = (isRotorInFront ? 0f : 45f) * Mathf.Deg2Rad;
            return new Vector3(
                Mathf.Cos(placementDegree + i * Mathf.PI / 2) * rotorDistance,
                0f,
                Mathf.Sin(placementDegree + i * Mathf.PI / 2) * rotorDistance
            );
        }).ToArray();

        ControlToRotorMatrix = new double[4, 4] {
            {thrustMultiplier, thrustMultiplier, thrustMultiplier, thrustMultiplier},
            {RotorPoses[0].x, RotorPoses[1].x, RotorPoses[2].x, RotorPoses[3].x},
            {-RotorPoses[0].z, -RotorPoses[1].z, -RotorPoses[2].z, -RotorPoses[3].z},
            {dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier}
        };
    }

    private void RunControlPipeline()
    {
        Rb.AddForce(-Rb.linearVelocity * Rb.linearVelocity.magnitude * airResistanceCoefficient);

        float desiredRoll = 0f, desiredPitch = 0f;
        if (CurrentTargetModifier != null)
        {
            CurrentTargetModifier.Modify(this, ref desiredRoll, ref desiredPitch);
        }

        float desiredYawVelocity = LeftJoystick.x * maxYawRate * Mathf.Deg2Rad;
        float desiredVerticalVelocity = LeftJoystick.y * maxVerticalVelocity;

        float upwardForce = (verticalGain * (desiredVerticalVelocity - CurrentVerticalVelocity) + Mass * Physics.gravity.magnitude) / Vector3.Dot(transform.up, Vector3.up);
        float rollControl = tiltGain * (desiredRoll - CurrentRoll) - tiltDamping * RollRate;
        float pitchControl = tiltGain * (desiredPitch - CurrentPitch) - tiltDamping * PitchRate;
        float yawControl = yawGain * (desiredYawVelocity - CurrentYawVelocity);

        float[] solution;
        if (CurrentRotorForceCalculator != null)
        {
            solution = CurrentRotorForceCalculator.Calculate(this, upwardForce, rollControl, pitchControl, yawControl);
        }
        else
        {
            Debug.LogWarning("No RotorForceCalculator assigned. Defaulting to basic calculation.");
            double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };
            solution = Matrix.Solve(ControlToRotorMatrix, controlInputArray).Select(x => (float)x).ToArray();
        }

        if (CurrentFluentRotor != null)
        {
            CurrentFluentRotor.Smooth(this, solution);
        }
        else
        {
            System.Array.Copy(solution, RotorForcesArray, solution.Length);
        }

        float[] finalRotorForces = (float[])RotorForcesArray.Clone();
        if (CurrentGroundEffect != null)
        {
            CurrentGroundEffect.PostProcess(this, ref finalRotorForces);
        }
        if (CurrentVelocityThrustInfluence != null)
        {
            CurrentVelocityThrustInfluence.PostProcess(this, ref finalRotorForces);
        }

        for (int i = 0; i < 4; i++)
        {
            Rb.AddForceAtPosition(transform.up * finalRotorForces[i], transform.TransformPoint(RotorPoses[i]));

            float sign = (i % 2 == 0) ? -1f : 1f;
            Rb.AddForceAtPosition(transform.forward * finalRotorForces[i] * sign * yawTorqueMultiplier, transform.TransformPoint(RotorPoses[i] + new Vector3(1, 0, 0)));
            Rb.AddForceAtPosition(-transform.forward * finalRotorForces[i] * sign * yawTorqueMultiplier, transform.TransformPoint(RotorPoses[i] - new Vector3(1, 0, 0)));
        }
    }


    public void Move(InputAction.CallbackContext context)
    {
        LeftJoystick = context.ReadValue<Vector2>();
    }

    public void Look(InputAction.CallbackContext context)
    {
        RightJoystick = context.ReadValue<Vector2>();
    }

    public void SwitchCamera(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            OnSwitchCamera?.Invoke();
        }
    }

    public void Restart(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            OnRestart?.Invoke();
        }
    }

    public void ToggleUI(InputAction.CallbackContext context)
    {
        if (context.performed)
        {
            OnToggleUI?.Invoke();
        }
    }
    
    public void ResetState()
    {
        CurrentTiltEstimator?.ResetState();
        CurrentTargetModifier?.ResetState();
        CurrentRotorForceCalculator?.ResetState();
        CurrentFluentRotor?.ResetState();
        CurrentGroundEffect?.ResetState();
        CurrentVelocityThrustInfluence?.ResetState();

        System.Array.Clear(RotorForcesArray, 0, RotorForcesArray.Length);
    }

    #region Strategy Setters
    public void SetTiltEstimator(TiltEstimator preset)
    {
        if (preset == null) return;
        CurrentTiltEstimator = Instantiate(preset);
        CurrentTiltEstimator.Initialize(this);
    }

    public void SetTargetModifier(TargetModifier preset)
    {
        if (preset == null) return;
        CurrentTargetModifier = Instantiate(preset);
    }

    public void SetRotorForceCalculator(RotorForceCalculator preset)
    {
        if (preset == null) return;
        CurrentRotorForceCalculator = Instantiate(preset);
    }

    public void SetFluentRotor(FluentRotorSmoother preset)
    {
        CurrentFluentRotor = preset != null ? Instantiate(preset) : null;
    }

    public void SetGroundEffect(GroundEffect preset)
    {
        CurrentGroundEffect = preset != null ? Instantiate(preset) : null;
    }

    public void SetVelocityThrustInfluence(VelocityThrustInfluence preset)
    {
        CurrentVelocityThrustInfluence = preset != null ? Instantiate(preset) : null;
    }
    #endregion
}