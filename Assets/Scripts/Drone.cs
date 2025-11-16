using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using Accord.Math;
using Accord.Math.Optimization;

using Vector3 = UnityEngine.Vector3;
using System;

[RequireComponent(typeof(Rigidbody))]
public class Drone : MonoBehaviour {
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
    [SerializeField] private bool doesVelocityInfluenceThrust = true;
    [Space]
    [SerializeField] private LineRenderer rotorLinePrefab;


    private float rollAngle { get { return Mathf.DeltaAngle(0f, rb.rotation.eulerAngles.z); } }
    private float pitchAngle { get { return Mathf.DeltaAngle(0f, rb.rotation.eulerAngles.x); } }

    private float currentRoll { get { return rollAngle * Mathf.Deg2Rad; } }
    private float currentPitch { get { return pitchAngle * Mathf.Deg2Rad; } }
    private float currentYawVelocity { get { return rb.angularVelocity.y * Mathf.Deg2Rad; } }
    private float currentVerticalVelocity { get { return rb.linearVelocity.y; } }

    private Vector3 localLinearVelocity { get { return Quaternion.Inverse(Quaternion.Euler(0, transform.eulerAngles.y, 0)) * rb.linearVelocity; } }
    private Vector3 localAngularVelocity { get { return transform.InverseTransformDirection(rb.angularVelocity); } }
    private float rollRate { get { return localAngularVelocity.z; } }
    private float pitchRate { get { return localAngularVelocity.x; } }

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

    private Vector2 leftJoystick;
    private Vector2 rightJoystick;

    private Rigidbody rb;
    private LineRenderer[] rotorLines;

    void Awake() {
        rb = GetComponent<Rigidbody>();
        rb.linearDamping = 0f;
        rotorLines = Enumerable.Range(0, 4).Select(i => Instantiate(rotorLinePrefab, transform)).ToArray();
    }

    void FixedUpdate() {
        rb.AddForce(-rb.linearVelocity * rb.linearVelocity.magnitude * airResistanceCoefficient * Time.fixedDeltaTime);
        // Debug.Log($"Roll: {rollAngle}, Pitch: {pitchAngle}, Yaw: {transform.eulerAngles.y}");
        double[,] controlToRotorMatrix = new double[4, 4] {
            {thrustMultiplier, thrustMultiplier, thrustMultiplier, thrustMultiplier},
            {rotorPoses[0].x, rotorPoses[1].x, rotorPoses[2].x, rotorPoses[3].x},
            {-rotorPoses[0].z, -rotorPoses[1].z, -rotorPoses[2].z, -rotorPoses[3].z},
            {dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier}
        };

        float desiredRoll;
        float desiredPitch;

        // Debug.Log($"{rightJoystick.x} {rightJoystick.y} X: {desiredRightVelocity:F2}, Z: {desiredForwardVelocity:F2}");
        if (isSupportOn && rightJoystick.magnitude < 0.1f) {
            float desiredRightVelocity = -localLinearVelocity.x;
            float desiredForwardVelocity = -localLinearVelocity.z;

            desiredRoll = Mathf.Atan(-desiredRightVelocity * supportGain) / (Mathf.PI / 2) * maxTiltAngle * Mathf.Deg2Rad;
            desiredPitch = Mathf.Atan(desiredForwardVelocity * supportGain) / (Mathf.PI / 2) * maxTiltAngle * Mathf.Deg2Rad;
        } else {
            desiredRoll = -rightJoystick.x * maxTiltAngle * Mathf.Deg2Rad;
            desiredPitch = rightJoystick.y * maxTiltAngle * Mathf.Deg2Rad;
        }

        float desiredYawVelocity = leftJoystick.x * maxYawRate * Mathf.Deg2Rad;
        float desiredVerticalVelocity = leftJoystick.y * maxVerticalVelocity;

        float rollControl = tiltGain * (desiredRoll - currentRoll) - tiltDamping * rollRate;
        float pitchControl = tiltGain * (desiredPitch - currentPitch) - tiltDamping * pitchRate;
        float yawControl = 40 * yawGain * (desiredYawVelocity - currentYawVelocity);
        float upwardForce = (verticalGain * (desiredVerticalVelocity - currentVerticalVelocity) + mass * Physics.gravity.magnitude) / Vector3.Dot(transform.up, Vector3.up);


        float[] solution;
        if (isRotorContrained) {
            upwardForce = Mathf.Clamp(upwardForce, 0, 4 * maxRotorForce);
            double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };
            double[,] H = Matrix.Dot(controlToRotorMatrix.Transpose(), controlToRotorMatrix).Multiply(2);
            double[] f = Matrix.Dot(controlToRotorMatrix.Transpose(), controlInputArray).Multiply(-2);
            QuadraticObjectiveFunction qof = new QuadraticObjectiveFunction(H, f);
            var cons = new System.Collections.Generic.List<LinearConstraint>();
            for (int i = 0; i < 4; i++) {
                var coeff = new double[4]; coeff[i] = 1.0;
                cons.Add(new LinearConstraint(4) {
                    CombinedAs = coeff,
                    ShouldBe = ConstraintType.GreaterThanOrEqualTo,
                    Value = 0.0
                });
                cons.Add(new LinearConstraint(4) {
                    CombinedAs = coeff,
                    ShouldBe = ConstraintType.LesserThanOrEqualTo,
                    Value = maxRotorForce
                });
            }
            var solver = new GoldfarbIdnani(qof, cons.ToArray());
            solver.Minimize();
            solution = solver.Solution.Select(x => (float)x).ToArray();
        } else {
            double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };
            solution = Matrix.Solve(controlToRotorMatrix, controlInputArray).Select(x => (float)x).ToArray();
        }

        if (isFluentRotor) {
            for (int i = 0; i < 4; i++) {
                rotorForcesArray[i] = Mathf.MoveTowards(rotorForcesArray[i], solution[i], maxRotorDelta);
            }
        } else {
            rotorForcesArray = solution;
        }

        if (isGroudEffectOn) {
            for (int i = 0; i < 4; i++) {
                rotorForcesArray[i] *= 1f + 6f * Mathf.Exp(-Mathf.Clamp(transform.position.y, 0, float.MaxValue) * 5f / rotorRadius);
            }
        }

        if (doesVelocityInfluenceThrust) {
            for (int i = 0; i < 4; i++) {
                Vector3 pv = rb.GetPointVelocity(transform.TransformPoint(rotorPoses[i]));
                Vector3 lpv = transform.InverseTransformDirection(pv);
                // Debug.Log($"Rotor {i} local vertical velocity: {lpv}");
                float rotorLocalVerticalVelocity = lpv.y;
                rotorForcesArray[i] *= 1f - rotorLocalVerticalVelocity / Mathf.Clamp(Mathf.Sqrt(Mathf.Abs(rotorForcesArray[i])), 0.1f, float.MaxValue);
                // rotorForcesArray[i] *= Mathf.Clamp01(1f - rotorLocalVerticalVelocity / 10f);
                Debug.DrawLine(transform.TransformPoint(rotorPoses[i]), transform.TransformPoint(rotorPoses[i]) + pv, Color.red);
            }
        }

        for (int i = 0; i < 4; i++) {
            float randomNoise = 1; // UnityEngine.Random.Range(0.9f, 1.1f);
            rb.AddForceAtPosition(transform.up * rotorForcesArray[i] * randomNoise, transform.TransformPoint(rotorPoses[i]));
            rb.AddForceAtPosition(transform.forward * rotorForcesArray[i] * randomNoise * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] + new Vector3(0.1f, 0, 0)));
            rb.AddForceAtPosition(-transform.forward * rotorForcesArray[i] * randomNoise * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] - new Vector3(0.1f, 0, 0)));
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
