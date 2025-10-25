using System;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class Drone : MonoBehaviour {
    [SerializeField] float angleMult = 10.0f;


    private Vector2 leftJoystick;
    private Vector2 rightJoystick;

    // private readonly float gravity = Physics.gravity.magnitude;
    private float mass = 0f;

    private Rigidbody rb;


    void Awake() {
        rb = GetComponent<Rigidbody>();
        mass = rb.mass;
    }

    void FixedUpdate() {
        Vector3 fRotorPos = new Vector3(0, 0, 1);
        Vector3 rRotorPos = new Vector3(1, 0, 0);
        Vector3 bRotorPos = new Vector3(0, 0, -1);
        Vector3 lRotorPos = new Vector3(-1, 0, 0);
        float kp = 1f;
        float kd = 1f;
        float ki = 0f;

        float thrustMultiplier = 1f;
        float dragMultiplier = 1f;

        float rotorDistance = fRotorPos.magnitude;


        float zAngle = (transform.eulerAngles.z + 180f) % 360f - 180f;
        float desiredRollRad = rightJoystick.x * 20f * Mathf.Deg2Rad;
        float currentRollRad = zAngle * Mathf.Deg2Rad;
        float rollRateRad = rb.angularVelocity.x;

        float rollControl = kp * (desiredRollRad - currentRollRad) - kd * rollRateRad;



        Matrix4x4 controlToRotor = new Matrix4x4(
            new Vector4(1, 1, 1, 1),
            new Vector4(0, rotorDistance, 0, -rotorDistance),
            new Vector4(rotorDistance, 0, -rotorDistance, 0),
            new Vector4(dragMultiplier, -dragMultiplier, dragMultiplier, -dragMultiplier)
        ).transpose;

        float upwardForce = mass * 9.81f;

        Vector4 controlInput = new(upwardForce, rollControl, 0f, 0f);

        Vector4 rotorForces = controlToRotor.inverse * controlInput;

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
