using System;
using UnityEditor.Rendering.Universal;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;

public class Drone : MonoBehaviour
{
    [SerializeField] Vector3 flMotor = new(-0.5f, 0.0f, 0.5f);
    [SerializeField] Vector3 frMotor = new(0.5f, 0.0f, 0.5f);
    [SerializeField] Vector3 blMotor = new(-0.5f, 0.0f, -0.5f);
    [SerializeField] Vector3 brMotor = new(0.5f, 0.0f, -0.5f);
    [SerializeField] float gravity = 9.81f;
    [SerializeField] float angleMult = 10.0f;

    Rigidbody rb;
    Vector2 leftJoystick; 
    Vector2 rightJoystick;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        float force = gravity / (4 * Vector3.Dot(transform.up, Vector3.up));

        Vector3 desiredDir = transform.InverseTransformDirection(-rb.linearVelocity) + new Vector3(rightJoystick.x, 0f, rightJoystick.y) * 4;`
        float zRad = (transform.eulerAngles.z + desiredDir.x * angleMult) * Mathf.Deg2Rad;
        float xRad = (transform.eulerAngles.x - desiredDir.z * angleMult) * Mathf.Deg2Rad;

        float flf = force * (1.0f + Mathf.Sin(zRad)) * (1.0f + Mathf.Sin(xRad));
        float frf = force * (1.0f - Mathf.Sin(zRad)) * (1.0f + Mathf.Sin(xRad));
        float blf = force * (1.0f + Mathf.Sin(zRad)) * (1.0f - Mathf.Sin(xRad));
        float brf = force * (1.0f - Mathf.Sin(zRad)) * (1.0f - Mathf.Sin(xRad));


        rb.AddForceAtPosition(transform.up * flf, transform.TransformPoint(flMotor));
        rb.AddForceAtPosition(transform.up * frf, transform.TransformPoint(frMotor));
        rb.AddForceAtPosition(transform.up * blf, transform.TransformPoint(blMotor));
        rb.AddForceAtPosition(transform.up * brf, transform.TransformPoint(brMotor));

        Debug.DrawLine(transform.position, transform.position + transform.TransformDirection(desiredDir), Color.red);
        Debug.DrawLine(transform.TransformPoint(flMotor), transform.TransformPoint(flMotor) + transform.up * flf);
        Debug.DrawLine(transform.TransformPoint(frMotor), transform.TransformPoint(frMotor) + transform.up * frf);
        Debug.DrawLine(transform.TransformPoint(blMotor), transform.TransformPoint(blMotor) + transform.up * blf);
        Debug.DrawLine(transform.TransformPoint(brMotor), transform.TransformPoint(brMotor) + transform.up * brf);
    }

    public void Move(InputAction.CallbackContext context)
    {
        leftJoystick = context.ReadValue<Vector2>();
    }

    public void Look(InputAction.CallbackContext context)
    {
        rightJoystick = context.ReadValue<Vector2>();
    }
}
