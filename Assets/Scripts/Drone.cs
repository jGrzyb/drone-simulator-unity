using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using Accord.Math;
using Accord.Math.Optimization;

using Vector3 = UnityEngine.Vector3;
using System;
using Unity.VisualScripting;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(PlayerInput))]
public class Drone : MonoBehaviour {
    [SerializeField] public ITiltEstimator tiltEstimatorPrefab;
    [SerializeField] public ITargetModifier targetModifierPrefab;
    [SerializeField] public IControlAllocator controlAllocatorPrefab;
    [SerializeField] public FluentRotors fluentRotorsPrefab;
    private ITiltEstimator tiltEstimator;
    private ITargetModifier targetModifier;
    private IControlAllocator controlAllocator;
    private FluentRotors fluentRotors;

    [SerializeField] private float maxTiltAngle = 30f;
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

    [Space]
    [SerializeField] private bool isRotorContrained = true;
    [SerializeField] private float maxRotorForce = 10.0f;

    [Space]
    [SerializeField] private bool isFluentRotor = true;
    [SerializeField] private float maxRotorDelta = 0.5f;

    [Space]
    [SerializeField] private bool isSupportOn = true;
    [SerializeField] private float supportGain = 0.5f;

    [Space]
    [SerializeField] private bool isGroudEffectOn = true;
    [SerializeField] private float rotorRadius = 0.2f;

    [Space]
    [SerializeField] private bool doesVelocityInfluenceThrust = false;
    [Space]
    [SerializeField] private LineRenderer rotorLinePrefab;


    private float currentRoll { get { return -tiltEstimator.GetRollAngle() * Mathf.Deg2Rad; } }
    private float currentPitch { get { return tiltEstimator.GetPitchAngle() * Mathf.Deg2Rad; } }
    private float currentYawVelocity { get { return transform.InverseTransformDirection(rb.angularVelocity).y * Mathf.Deg2Rad; } }
    private float currentVerticalVelocity { get { return rb.linearVelocity.y; } }

    private float rollRate { get { return -tiltEstimator.GetRollRate() * Mathf.Deg2Rad; } }
    private float pitchRate { get { return tiltEstimator.GetPitchRate() * Mathf.Deg2Rad; } }

    private float mass { get { return rb?.mass ?? 0f; } }

    private Vector3[] rotorPoses {
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

    private float[] rotorForcesArray = new float[4];

    public Vector2 leftJoystick { get; private set; }
    public Vector2 rightJoystick { get; private set; }

    private Rigidbody rb;
    private LineRenderer[] rotorLines;

    void Awake() {
        rb = GetComponent<Rigidbody>();
        tiltEstimator = Instantiate(tiltEstimatorPrefab);
        tiltEstimator.Initialize(this);
        targetModifier = Instantiate(targetModifierPrefab);
        targetModifier.Initialize(this);
        controlAllocator = Instantiate(controlAllocatorPrefab);
        fluentRotors = Instantiate(fluentRotorsPrefab);
        rb.linearDamping = 0f;
        rotorLines = Enumerable.Range(0, 4).Select(i => Instantiate(rotorLinePrefab, transform)).ToArray();
    }

    void FixedUpdate() {
        tiltEstimator.UpdateFilter();
        rb.AddForce(-rb.linearVelocity * rb.linearVelocity.magnitude * airResistanceCoefficient * Time.fixedDeltaTime);
        double[,] controlToRotorMatrix = new double[4, 4] {
            {thrustMultiplier, thrustMultiplier, thrustMultiplier, thrustMultiplier},
            {rotorPoses[0].x, rotorPoses[1].x, rotorPoses[2].x, rotorPoses[3].x},
            {-rotorPoses[0].z, -rotorPoses[1].z, -rotorPoses[2].z, -rotorPoses[3].z},
            {dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier}
        };

        DesiredTilt desiredTilt = targetModifier.GetDesiredTilt();
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

        fluentRotors?.ModifyRotorForces(ref rotorForcesArray, solution);

        float[] trueRotorForces = rotorForcesArray.Clone() as float[];
        if (isGroudEffectOn) {
            for (int i = 0; i < 4; i++) {
                trueRotorForces[i] *= 1f + 6f * Mathf.Exp(-Mathf.Max(transform.position.y, 0) * 5f / rotorRadius);
            }
        }

        if (doesVelocityInfluenceThrust) {
            for (int i = 0; i < 4; i++) {
                Vector3 pv = rb.GetPointVelocity(transform.TransformPoint(rotorPoses[i]));
                Vector3 lpv = transform.InverseTransformDirection(pv);
                // Debug.Log($"Rotor {i} local vertical velocity: {lpv}");
                float rotorLocalVerticalVelocity = lpv.y;
                trueRotorForces[i] *= 1f - rotorLocalVerticalVelocity / Mathf.Max(Mathf.Sqrt(Mathf.Abs(trueRotorForces[i])), 0.1f);
                Debug.DrawLine(transform.TransformPoint(rotorPoses[i]), transform.TransformPoint(rotorPoses[i]) + pv, Color.red);
            }
        }

        for (int i = 0; i < 4; i++) {
            float randomNoise = 1; // UnityEngine.Random.Range(0.9f, 1.1f);
            rb.AddForceAtPosition(transform.up * trueRotorForces[i] * randomNoise, transform.TransformPoint(rotorPoses[i]));
            rb.AddForceAtPosition(transform.forward * trueRotorForces[i] * randomNoise * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] + new Vector3(0.1f, 0, 0)));
            rb.AddForceAtPosition(-transform.forward * trueRotorForces[i] * randomNoise * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] - new Vector3(0.1f, 0, 0)));
        }
    }

    public void Move(InputAction.CallbackContext context) {
        leftJoystick = context.ReadValue<Vector2>();
    }

    public void Look(InputAction.CallbackContext context) {
        rightJoystick = context.ReadValue<Vector2>();
    }

    void Update() {
        for (int i = 0; i < 4; i++) {
            rotorLines[i].SetPosition(0, transform.TransformPoint(rotorPoses[i]));
            rotorLines[i].SetPosition(1, transform.TransformPoint(rotorPoses[i] + Vector3.up * rotorForcesArray[i] * 5f));
        }
    }
}
