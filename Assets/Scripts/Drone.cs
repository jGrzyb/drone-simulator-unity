using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using Accord.Math;
using Accord.Math.Optimization;

using Vector3 = UnityEngine.Vector3;

public class Drone : MonoBehaviour {
    [SerializeField] private float maxTiltAngle = 20f;
    [SerializeField] private float kp = 1f;
    [SerializeField] private float maxYawRate = 1f;
    [SerializeField] private float kd = 1f;
    [SerializeField] private float ki = 1f;
    [SerializeField] private float thrustMultiplier = 1f;
    [SerializeField] private float dragMultiplier = 1f;
    [SerializeField] private float rotorDistance = 0.5f;
    [SerializeField] private float maxRotorForce = 50.0f;
    [SerializeField] private bool isRotorInFront = true;


    private float rollAngle { get { return (transform.eulerAngles.z + 180f) % 360f - 180f; } }
    private float pitchAngle { get { return (transform.eulerAngles.x + 180f) % 360f - 180f; } }

    private float desiredRoll { get { return -rightJoystick.x * maxTiltAngle * Mathf.Deg2Rad; } }
    private float desiredPitch { get { return rightJoystick.y * maxTiltAngle * Mathf.Deg2Rad; } }
    private float desiredYawVelocity { get { return leftJoystick.x * maxYawRate * Mathf.Deg2Rad; } }
    private float desiredVerticalVelocity { get { return leftJoystick.y * Physics.gravity.magnitude / 2; } }

    private float currentRoll { get { return rollAngle * Mathf.Deg2Rad; } }
    private float currentPitch { get { return pitchAngle * Mathf.Deg2Rad; } }
    private float currentYawVelocity { get { return rb.angularVelocity.y * Mathf.Deg2Rad; } }
    private float currentVerticalVelocity { get { return rb.linearVelocity.y; } }

    private Vector3 localAngularVelocity { get { return transform.InverseTransformDirection(rb.angularVelocity); } }
    private float rollRate { get { return localAngularVelocity.z; } }
    private float pitchRate { get { return localAngularVelocity.x; } }

    private float mass { get { return rb?.mass ?? 0f; } }

    private Vector2 leftJoystick;
    private Vector2 rightJoystick;

    private Rigidbody rb;


    void Awake() {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate() {
        float placementDegree = (isRotorInFront ? 0f : 45f) * Mathf.Deg2Rad;
        Vector3[] rotorPoses = Enumerable.Range(0, 4).Select(i =>
            new Vector3(
                Mathf.Cos(placementDegree + i * Mathf.PI / 2) * rotorDistance,
                0f,
                Mathf.Sin(placementDegree + i * Mathf.PI / 2) * rotorDistance
            )
        ).ToArray();

        double[,] controlToRotorArray = new double[4, 4] {
            {1, 1, 1, 1},
            {rotorPoses[0].x, rotorPoses[1].x, rotorPoses[2].x, rotorPoses[3].x},
            {-rotorPoses[0].z, -rotorPoses[1].z, -rotorPoses[2].z, -rotorPoses[3].z},
            {dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier}
        };

        float rollControl = kp * (desiredRoll - currentRoll) - kd * rollRate;
        float pitchControl = kp * (desiredPitch - currentPitch) - kd * pitchRate;
        float yawControl = 40 * kp * (desiredYawVelocity - currentYawVelocity);
        float upwardForce = (kp * (desiredVerticalVelocity - currentVerticalVelocity) + mass * Physics.gravity.magnitude) / Vector3.Dot(transform.up, Vector3.up);


        double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };
        double[,] H = Matrix.Dot(controlToRotorArray.Transpose(), controlToRotorArray).Multiply(2);
        double[] f = Matrix.Dot(controlToRotorArray.Transpose(), controlInputArray).Multiply(-2);
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
        double[] rotorForcesArray = solver.Solution;

        for (int i = 0; i < 4; i++) {
            rb.AddForceAtPosition(transform.up * (float)rotorForcesArray[i], transform.TransformPoint(rotorPoses[i]));
            rb.AddForceAtPosition(transform.forward * (float)rotorForcesArray[i] * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] + new Vector3(0.1f, 0, 0)));
            rb.AddForceAtPosition(-transform.forward * (float)rotorForcesArray[i] * ((i % 2 * 2) - 1), transform.TransformPoint(rotorPoses[i] - new Vector3(0.1f, 0, 0)));

            Debug.DrawLine(transform.TransformPoint(rotorPoses[i]), transform.TransformPoint(rotorPoses[i]) + transform.up * (float)rotorForcesArray[i]);
        }
    }

    public void Move(InputAction.CallbackContext context) {
        leftJoystick = context.ReadValue<Vector2>();
    }

    public void Look(InputAction.CallbackContext context) {
        rightJoystick = context.ReadValue<Vector2>();
    }
}
