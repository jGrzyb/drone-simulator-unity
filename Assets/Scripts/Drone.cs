using System;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class Drone : MonoBehaviour {
    [SerializeField] private float maxTiltAngle = 20f;
    [SerializeField] private float kp = 1f;
    [SerializeField] private float kd = 1f;
    [SerializeField] private float ki = 1f;
    [SerializeField] private float thrustMultiplier = 1f;
    [SerializeField] private float dragMultiplier = 1f;
    [SerializeField] private float rotorDistance = 0.5f;
    [SerializeField] private bool isRotorInFront = true;


    private float rollAngle { get { return (transform.eulerAngles.z + 180f) % 360f - 180f; } }
    private float pitchAngle { get { return (transform.eulerAngles.x + 180f) % 360f - 180f; } }
    private float yawAngle { get { return (transform.eulerAngles.y + 180f) % 360f - 180f; } }

    private float desiredRoll { get { return -rightJoystick.x * maxTiltAngle * Mathf.Deg2Rad; } }
    private float desiredPitch { get { return rightJoystick.y * maxTiltAngle * Mathf.Deg2Rad; } }
    private float desiredYaw { get { return leftJoystick.x * maxTiltAngle * Mathf.Deg2Rad; } }
    private float desiredVerticalVelocity { get { return leftJoystick.y * Physics.gravity.magnitude / 2; } }

    private float currentRoll { get { return rollAngle * Mathf.Deg2Rad; } }
    private float currentPitch { get { return pitchAngle * Mathf.Deg2Rad; } }
    private float currentYaw { get { return yawAngle * Mathf.Deg2Rad; } }

    Vector3 localAngularVelocity { get { return transform.InverseTransformDirection(rb.angularVelocity); } }
    private float rollRate { get { return localAngularVelocity.z; } }
    private float pitchRate { get { return localAngularVelocity.x; } }
    private float yawRate { get { return localAngularVelocity.y; } }
    private float currentVerticalVelocity { get { return rb.linearVelocity.y; } }

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

        Matrix4x4 controlToRotor = new Matrix4x4(
            new Vector4(1, 1, 1, 1),
            new Vector4(rotorPoses[0].x, rotorPoses[1].x, rotorPoses[2].x, rotorPoses[3].x),
            new Vector4(-rotorPoses[0].z, -rotorPoses[1].z, -rotorPoses[2].z, -rotorPoses[3].z),
            new Vector4(dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier)
        ).transpose.inverse;

        float rollControl = kp * (desiredRoll - currentRoll) - kd * rollRate;
        float pitchControl = kp * (desiredPitch - currentPitch) - kd * pitchRate;
        float yawControl = kp * (desiredYaw - currentYaw) - kd * yawRate;
        float upwardForce = (kp * (desiredVerticalVelocity - currentVerticalVelocity) + mass * Physics.gravity.magnitude) / Vector3.Dot(transform.up, Vector3.up);

        Vector4 controlInput = new(upwardForce, rollControl, pitchControl, 0f);
        Vector4 rotorForcesVector = controlToRotor * controlInput;
        float[] rotorForces = new float[] { rotorForcesVector.x, rotorForcesVector.y, rotorForcesVector.z, rotorForcesVector.w };

        for (int i = 0; i < 4; i++) {
            rb.AddForceAtPosition(transform.up * rotorForces[i], transform.TransformPoint(rotorPoses[i]));
            Debug.DrawLine(transform.TransformPoint(rotorPoses[i]), transform.TransformPoint(rotorPoses[i]) + transform.up * rotorForces[i]);
        }
    }

    public void Move(InputAction.CallbackContext context) {
        leftJoystick = context.ReadValue<Vector2>();
    }

    public void Look(InputAction.CallbackContext context) {
        rightJoystick = context.ReadValue<Vector2>();
    }
}
