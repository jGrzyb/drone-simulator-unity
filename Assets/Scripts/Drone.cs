using System.Collections.Generic;
using System.Linq;
using UnityEngine;
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
    [SerializeField] private float tiltGain = 5f;
    [SerializeField] private float tiltDamping = 1f;
    [Space]
    [SerializeField] private float maxYawRate = 2f;
    [SerializeField] private float yawGain = 5f;
    [Space]
    [SerializeField] private float maxVerticalVelocity = 5f;
    [SerializeField] private float verticalGain = 1f;
    [Space]
    [SerializeField] private float thrustMultiplier = 1f;
    [SerializeField] private float dragMultiplier = 1f;
    [SerializeField] private float airResistanceCoefficient = 1f;
    [Space]
    [SerializeField] private float rotorDistance = 0.5f;
    [SerializeField] private bool isRotorInFront = true;

    [Header("Visuals")]
    [SerializeField] private LineRenderer rotorLinePrefab;

    public Rigidbody Rb { get; private set; }
    public Vector2 LeftJoystick { get; private set; }
    public Vector2 RightJoystick { get; private set; }
    public Vector3 LocalLinearVelocity => Quaternion.Inverse(Quaternion.Euler(0, transform.eulerAngles.y, 0)) * Rb.linearVelocity;
    public float CurrentRoll => -CurrentTiltEstimator.RollAngle * Mathf.Deg2Rad;
    public float CurrentPitch => CurrentTiltEstimator.PitchAngle * Mathf.Deg2Rad;
    public float RollRate => -CurrentTiltEstimator.RollRate * Mathf.Deg2Rad;
    public float PitchRate => CurrentTiltEstimator.PitchRate * Mathf.Deg2Rad;
    public float CurrentYawVelocity => transform.InverseTransformDirection(Rb.angularVelocity).y * Mathf.Deg2Rad;
    public float CurrentVerticalVelocity => Rb.linearVelocity.y;
    public float Mass => Rb?.mass ?? 0f;
    public Vector3[] RotorPoses =>
        Enumerable.Range(0, 4).Select(i => {
            float placementDegree = (isRotorInFront ? 0f : 45f) * Mathf.Deg2Rad;
            return new Vector3(
                Mathf.Cos(placementDegree + i * Mathf.PI / 2) * rotorDistance,
                0f,
                Mathf.Sin(placementDegree + i * Mathf.PI / 2) * rotorDistance
            );
        }).ToArray();
    public double[,] ControlToRotorMatrix => new double[4, 4] {
        {thrustMultiplier, thrustMultiplier, thrustMultiplier, thrustMultiplier},
        {RotorPoses[0].x, RotorPoses[1].x, RotorPoses[2].x, RotorPoses[3].x},
        {-RotorPoses[0].z, -RotorPoses[1].z, -RotorPoses[2].z, -RotorPoses[3].z},
        {dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier}
    };

    [HideInInspector] public float[] RotorForcesArray = new float[4];

    public TiltEstimator CurrentTiltEstimator { get; private set; }
    public TargetModifier CurrentTargetModifier { get; private set; }
    public RotorForceCalculator CurrentRotorForceCalculator { get; private set; }
    public FluentRotorSmoother CurrentFluentRotor { get; private set; }
    public GroundEffect CurrentGroundEffect { get; private set; }
    public VelocityThrustInfluence CurrentVelocityThrustInfluence { get; private set; }

    private LineRenderer[] rotorLines;

    void Awake()
    {
        Rb = GetComponent<Rigidbody>();
        rotorLines = Enumerable.Range(0, 4).Select(i => Instantiate(rotorLinePrefab, transform)).ToArray();

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
        if (CurrentTiltEstimator != null)
        {
            CurrentTiltEstimator.Estimate(this);
        }
        else
        {
            Debug.LogError("No TiltEstimator assigned to the drone!");
            return;
        }

        Rb.AddForce(-Rb.linearVelocity * Rb.linearVelocity.magnitude * airResistanceCoefficient * Time.fixedDeltaTime);


        float desiredRoll = 0f, desiredPitch = 0f;
        if (CurrentTargetModifier != null)
        {
            CurrentTargetModifier.Modify(this, ref desiredRoll, ref desiredPitch);
        }

        float desiredYawVelocity = LeftJoystick.x * maxYawRate * Mathf.Deg2Rad;
        float desiredVerticalVelocity = LeftJoystick.y * maxVerticalVelocity;

        float rollControl = tiltGain * (desiredRoll - CurrentRoll) - tiltDamping * RollRate;
        float pitchControl = tiltGain * (desiredPitch - CurrentPitch) - tiltDamping * PitchRate;
        float yawControl = 40 * yawGain * (desiredYawVelocity - CurrentYawVelocity);
        float upwardForce = (verticalGain * (desiredVerticalVelocity - CurrentVerticalVelocity) + Mass * Physics.gravity.magnitude) / Vector3.Dot(transform.up, Vector3.up);

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

        float[] smoothedForces = (CurrentFluentRotor != null) ? CurrentFluentRotor.Smooth(this, solution) : solution;

        float[] finalRotorForces = (float[])smoothedForces.Clone();
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

            float offset = dragMultiplier / 2.0f;
            float sign = (i % 2 == 0) ? -1f : 1f;

            Rb.AddForceAtPosition(transform.forward * finalRotorForces[i] * sign, transform.TransformPoint(RotorPoses[i] + new Vector3(offset, 0, 0)));
            Rb.AddForceAtPosition(-transform.forward * finalRotorForces[i] * sign, transform.TransformPoint(RotorPoses[i] - new Vector3(offset, 0, 0)));
        }
    }

    void Update()
    {
        for (int i = 0; i < 4; i++)
        {
            rotorLines[i].SetPosition(0, transform.TransformPoint(RotorPoses[i]));
            rotorLines[i].SetPosition(1, transform.TransformPoint(RotorPoses[i] + Vector3.up * RotorForcesArray[i] * 0.1f));
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
}