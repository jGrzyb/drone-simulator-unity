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
    [SerializeField] private float rotorDistance = 1f;
    [SerializeField] private Vector3 fRotorPos = new Vector3(0, 0, 1);
    [SerializeField] private Vector3 rRotorPos = new Vector3(1, 0, 0);
    [SerializeField] private Vector3 bRotorPos = new Vector3(0, 0, -1);
    [SerializeField] private Vector3 lRotorPos = new Vector3(-1, 0, 0);

    private float rollAngle { get { return (transform.eulerAngles.z + 180f) % 360f - 180f; } }
    private float pitchAngle { get { return (transform.eulerAngles.x + 180f) % 360f - 180f; } }
    private float yawAngle { get { return (transform.eulerAngles.y + 180f) % 360f - 180f; } }

    private float desiredRoll { get { return -rightJoystick.x * maxTiltAngle * Mathf.Deg2Rad; } }
    private float desiredPitch { get { return rightJoystick.y * maxTiltAngle * Mathf.Deg2Rad; } }
    private float desiredYaw { get { return leftJoystick.x * maxTiltAngle * Mathf.Deg2Rad; } }

    private float currentRoll { get { return rollAngle * Mathf.Deg2Rad; } }
    private float currentPitch { get { return pitchAngle * Mathf.Deg2Rad; } }
    private float currentYaw { get { return yawAngle * Mathf.Deg2Rad; } }

    private float rollRate { get { return rb.angularVelocity.z; } }
    private float pitchRate { get { return rb.angularVelocity.x; } }
    private float yawRate { get { return rb.angularVelocity.y; } }

    private float upwardForce { get { return mass * Physics.gravity.magnitude; } }

    private float mass { get { return rb?.mass ?? 0f; } }

    private Vector2 leftJoystick;
    private Vector2 rightJoystick;

    private Rigidbody rb;


    void Awake() {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate() {
        Matrix4x4 controlToRotor = new Matrix4x4(
            new Vector4(1, 1, 1, 1),
            new Vector4(0, rotorDistance, 0, -rotorDistance),
            new Vector4(-rotorDistance, 0, rotorDistance, 0),
            new Vector4(dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier)
        ).transpose.inverse;

        float rollControl = kp * (desiredRoll - currentRoll) - kd * rollRate;
        float pitchControl = kp * (desiredPitch - currentPitch) - kd * pitchRate;
        float yawControl = kp * (desiredYaw - currentYaw) - kd * yawRate;

        Debug.Log($"{rb.angularVelocity.z}    {rollRate}    {rollControl}");


        Vector4 controlInput = new(upwardForce, rollControl, pitchControl, 0f);
        Vector4 rotorForces = controlToRotor * controlInput;

        rb.AddForceAtPosition(transform.up * rotorForces.x, transform.TransformPoint(fRotorPos));
        rb.AddForceAtPosition(transform.up * rotorForces.y, transform.TransformPoint(rRotorPos));
        rb.AddForceAtPosition(transform.up * rotorForces.z, transform.TransformPoint(bRotorPos));
        rb.AddForceAtPosition(transform.up * rotorForces.w, transform.TransformPoint(lRotorPos));

        Debug.DrawLine(transform.TransformPoint(fRotorPos), transform.TransformPoint(fRotorPos) + transform.up * rotorForces.x);
        Debug.DrawLine(transform.TransformPoint(rRotorPos), transform.TransformPoint(rRotorPos) + transform.up * rotorForces.y);
        Debug.DrawLine(transform.TransformPoint(bRotorPos), transform.TransformPoint(bRotorPos) + transform.up * rotorForces.z);
        Debug.DrawLine(transform.TransformPoint(lRotorPos), transform.TransformPoint(lRotorPos) + transform.up * rotorForces.w);
    }

    public void Move(InputAction.CallbackContext context) {
        leftJoystick = context.ReadValue<Vector2>();
    }

    public void Look(InputAction.CallbackContext context) {
        rightJoystick = context.ReadValue<Vector2>();
    }
}
