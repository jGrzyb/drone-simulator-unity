using System.Linq;
using UnityEngine;
using Accord.Math;

using Vector3 = UnityEngine.Vector3;
using System;

[RequireComponent(typeof(Rigidbody))]
public class Drone : MonoBehaviour {
    [SerializeField] private KalmanTiltEstimator kalmanTiltEstimatorComponent;
    [SerializeField] private TransformTiltEstimator transformTiltEstimatorComponent;
    [SerializeField] private SupportTargetModifier supportTargetModifierComponent;
    [SerializeField] private TiltTargetModifier tiltTargetModifierComponent;
    [SerializeField] private ConstrainedControlAllocator constrainedControlAllocatorComponent;
    [SerializeField] private UnconstrainedControlAllocator unconstrainedControlAllocatorComponent;
    [SerializeField] private FluentRotors fluentRotorsComponent;
    [SerializeField] private GroundEffect groundEffectComponent;
    [SerializeField] private VelocityDependent velocityDependentComponent;

    private ITiltEstimator tiltEstimator;
    private ITargetModifier targetModifier;
    private IControlAllocator controlAllocator;
    private FluentRotors fluentRotors;
    private GroundEffect groundEffect;
    private VelocityDependent velocityDependent;

    [SerializeField] public float maxTiltAngle = 30f;
    [SerializeField] public float tiltGain = 5f;
    [SerializeField] public float tiltDamping = 1f;

    [Space]
    [SerializeField] public float maxYawRate = 2f;
    [SerializeField] public float yawGain = 5f;

    [Space]
    [SerializeField] public float maxVerticalVelocity = 5f;
    [SerializeField] public float verticalGain = 1f;

    [Space]
    [SerializeField] public float thrustMultiplier = 1f;
    [SerializeField] public float dragMultiplier = 1f;
    [SerializeField] public float airResistanceCoefficient = 0.02f;

    [Space]
    [SerializeField] public float rotorDistance = 0.5f;
    [SerializeField] public bool isRotorInFront = true;


    private float currentRoll { get { return -tiltEstimator.GetRollAngle() * Mathf.Deg2Rad; } }
    private float currentPitch { get { return tiltEstimator.GetPitchAngle() * Mathf.Deg2Rad; } }
    private float currentYawVelocity { get { return transform.InverseTransformDirection(rb.angularVelocity).y * Mathf.Deg2Rad; } }
    private float currentVerticalVelocity { get { return rb.linearVelocity.y; } }

    private float rollRate { get { return -tiltEstimator.GetRollRate() * Mathf.Deg2Rad; } }
    private float pitchRate { get { return tiltEstimator.GetPitchRate() * Mathf.Deg2Rad; } }

    private float mass { get { return rb.mass; } }

    private Vector2 leftJoystick { get { return InputManager.I.leftJoystick; } }
    private Vector3 initialPosition;

    public Vector3[] rotorPoses {
        get {
            float placementDegree = (isRotorInFront ? 0f : 45f) * Mathf.Deg2Rad;
            return Enumerable.Range(0, 4).Select(i =>
                new Vector3(
                    Mathf.Cos(placementDegree + i * Mathf.PI / 2) * rotorDistance,
                    0f,
                    Mathf.Sin(placementDegree + i * Mathf.PI / 2) * rotorDistance
                )
            ).ToArray();
        }
    }


    private Rigidbody rb;
    public float[] rotorForcesArray { get; private set; } = new float[4];

    void Awake() {
        initialPosition = transform.position;
        rb = GetComponent<Rigidbody>();
        rb.linearDamping = 0f;

        UIManager.I.maxTiltAngleSlider.onValueChanged.AddListener(value => maxTiltAngle = value);
        UIManager.I.tiltGainSlider.onValueChanged.AddListener(value => tiltGain = value);
        UIManager.I.tiltDampingSlider.onValueChanged.AddListener(value => tiltDamping = value);
        UIManager.I.maxYawRateSlider.onValueChanged.AddListener(value => maxYawRate = value);
        UIManager.I.yawGainSlider.onValueChanged.AddListener(value => yawGain = value);
        UIManager.I.maxVerticalVelocitySlider.onValueChanged.AddListener(value => maxVerticalVelocity = value);
        UIManager.I.verticalGainSlider.onValueChanged.AddListener(value => verticalGain = value);
        UIManager.I.airResistanceSlider.onValueChanged.AddListener(value => airResistanceCoefficient = value);
        UIManager.I.rotorDistanceSlider.onValueChanged.AddListener(value => rotorDistance = value);
        UIManager.I.isRotorInFrontToggle.onValueChanged.AddListener(value => isRotorInFront = value);

        kalmanTiltEstimatorComponent = Instantiate(kalmanTiltEstimatorComponent);
        kalmanTiltEstimatorComponent.Initialize(this);
        transformTiltEstimatorComponent = Instantiate(transformTiltEstimatorComponent);
        transformTiltEstimatorComponent.Initialize(this);
        UIManager.I.onKalmanSelected.AddListener(() => tiltEstimator = kalmanTiltEstimatorComponent);
        UIManager.I.onTransformSelected.AddListener(() => tiltEstimator = transformTiltEstimatorComponent);

        supportTargetModifierComponent = Instantiate(supportTargetModifierComponent);
        supportTargetModifierComponent.Initialize(this);
        tiltTargetModifierComponent = Instantiate(tiltTargetModifierComponent);
        tiltTargetModifierComponent.Initialize(this);
        UIManager.I.onSupportTargetSelected.AddListener(() => targetModifier = supportTargetModifierComponent);
        UIManager.I.onTiltTargetSelected.AddListener(() => targetModifier = tiltTargetModifierComponent);

        constrainedControlAllocatorComponent = Instantiate(constrainedControlAllocatorComponent);
        constrainedControlAllocatorComponent.Initialize();
        unconstrainedControlAllocatorComponent = Instantiate(unconstrainedControlAllocatorComponent);
        unconstrainedControlAllocatorComponent.Initialize();
        UIManager.I.onConstrainedSelected.AddListener(() => controlAllocator = constrainedControlAllocatorComponent);
        UIManager.I.onUnconstrainedSelected.AddListener(() => controlAllocator = unconstrainedControlAllocatorComponent);

        fluentRotorsComponent = Instantiate(fluentRotorsComponent);
        fluentRotorsComponent.Initialize();
        UIManager.I.onFluentOn.AddListener(() => fluentRotors = fluentRotorsComponent);
        UIManager.I.onFluentOff.AddListener(() => fluentRotors = null);

        groundEffectComponent = Instantiate(groundEffectComponent);
        groundEffectComponent.Initialize();
        UIManager.I.onGroundOn.AddListener(() => groundEffect = groundEffectComponent);
        UIManager.I.onGroundOff.AddListener(() => groundEffect = null);

        velocityDependentComponent = Instantiate(velocityDependentComponent);
        UIManager.I.onVelDependentOn.AddListener(() => velocityDependent = velocityDependentComponent);
        UIManager.I.onVelDependentOff.AddListener(() => velocityDependent = null);
    }

    void Start() {
        
    }

    void FixedUpdate() {
        tiltEstimator.UpdateFilter();
        rb.AddForce(-rb.linearVelocity * rb.linearVelocity.magnitude * airResistanceCoefficient );
        double[,] controlToRotorMatrix = new double[4, 4] {
            {thrustMultiplier, thrustMultiplier, thrustMultiplier, thrustMultiplier},
            {rotorPoses[0].x, rotorPoses[1].x, rotorPoses[2].x, rotorPoses[3].x},
            {-rotorPoses[0].z, -rotorPoses[1].z, -rotorPoses[2].z, -rotorPoses[3].z},
            {dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier}
        };

        DesiredTilt desiredTilt = targetModifier.GetDesiredTilt() * maxTiltAngle * Mathf.Deg2Rad;
        float desiredRoll = desiredTilt.desiredRoll;
        float desiredPitch = desiredTilt.desiredPitch;

        float desiredYawVelocity = leftJoystick.x * maxYawRate * Mathf.Deg2Rad;
        float desiredVerticalVelocity = leftJoystick.y * maxVerticalVelocity;

        float rollControl = tiltGain * (desiredRoll - currentRoll) - tiltDamping * rollRate;
        float pitchControl = tiltGain * (desiredPitch - currentPitch) - tiltDamping * pitchRate;
        float yawControl = 40 * yawGain * (desiredYawVelocity - currentYawVelocity);
        float upwardForce = (verticalGain * (desiredVerticalVelocity - currentVerticalVelocity) + mass * Physics.gravity.magnitude) / Vector3.Dot(transform.up, Vector3.up);


        float[] solution = controlAllocator.Allocate(
            upwardForce,
            rollControl,
            pitchControl,
            yawControl,
            controlToRotorMatrix
        );

        rotorForcesArray = fluentRotors?.GetModifiedRotorForces(rotorForcesArray, solution) ?? solution;

        float[] appliedRotorForces = (float[])rotorForcesArray.Clone();
        groundEffect?.ModifyAppliedForces(ref appliedRotorForces, transform.position);
        velocityDependent?.ModifyAppliedForces(ref appliedRotorForces, rb, transform, rotorPoses);

        for (int i = 0; i < 4; i++) {
            float randomNoise = 1; // UnityEngine.Random.Range(0.9f, 1.1f);
            rb.AddForceAtPosition(transform.up * appliedRotorForces[i] * randomNoise, transform.TransformPoint(rotorPoses[i]));
            rb.AddForceAtPosition(transform.forward * appliedRotorForces[i] * randomNoise * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] + new Vector3(0.1f, 0, 0)));
            rb.AddForceAtPosition(-transform.forward * appliedRotorForces[i] * randomNoise * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] - new Vector3(0.1f, 0, 0)));
        }
    }

    public void ResetDronePosition() {
        StartCoroutine(ResetDronePositionCoroutine());
    }

    private System.Collections.IEnumerator ResetDronePositionCoroutine() {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        rb.isKinematic = true;

        yield return new WaitForFixedUpdate();

        transform.position = initialPosition;
        transform.rotation = Quaternion.identity;

        yield return new WaitForFixedUpdate();
        
        rb.isKinematic = false;
        rb.WakeUp();
    }
}
