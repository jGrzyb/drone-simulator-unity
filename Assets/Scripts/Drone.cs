using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class Drone : MonoBehaviour {
    [SerializeField] float angleMult = 10.0f;


    private Vector2 leftJoystick;
    private Vector2 rightJoystick;

    private readonly float gravity = Physics.gravity.magnitude;
    private float mass = 0f;

    private Rigidbody rb;

    private Rotor[] rotors;


    void Awake() {
        rb = GetComponent<Rigidbody>();
        rotors = GetComponentsInChildren<Rotor>();
        mass = rb.mass + rotors.Sum(x => x.mass);
    }

    void FixedUpdate() {

        var droneRotationY = Quaternion.Euler(0, transform.eulerAngles.y, 0);
        Vector3 desiredDir = Quaternion.Inverse(droneRotationY) * -rb.linearVelocity + new Vector3(rightJoystick.x, leftJoystick.y, rightJoystick.y) * 4;
        float zRad = (transform.eulerAngles.z + desiredDir.x * angleMult) * Mathf.Deg2Rad;
        float xRad = (transform.eulerAngles.x - desiredDir.z * angleMult) * Mathf.Deg2Rad;

        float deviation = Vector3.Dot(transform.up, Vector3.up);
        float thrustMultiplier = 1f + desiredDir.y;
        float force = mass * gravity * thrustMultiplier / 4f;

        float flf = force * (1.0f + Mathf.Sin(zRad)) * (1.0f + Mathf.Sin(xRad));
        float frf = force * (1.0f - Mathf.Sin(zRad)) * (1.0f + Mathf.Sin(xRad));
        float blf = force * (1.0f + Mathf.Sin(zRad)) * (1.0f - Mathf.Sin(xRad));
        float brf = force * (1.0f - Mathf.Sin(zRad)) * (1.0f - Mathf.Sin(xRad));


        // rb.AddForceAtPosition(transform.up * flf, transform.TransformPoint(flMotor));
        // rb.AddForceAtPosition(transform.up * frf, transform.TransformPoint(frMotor));
        // rb.AddForceAtPosition(transform.up * blf, transform.TransformPoint(blMotor));
        // rb.AddForceAtPosition(transform.up * brf, transform.TransformPoint(brMotor));

        // Debug.DrawLine(transform.position, transform.position + droneRotationY * desiredDir, Color.red);
        // Debug.DrawLine(transform.TransformPoint(flMotor), transform.TransformPoint(flMotor) + transform.up * flf);
        // Debug.DrawLine(transform.TransformPoint(frMotor), transform.TransformPoint(frMotor) + transform.up * frf);
        // Debug.DrawLine(transform.TransformPoint(blMotor), transform.TransformPoint(blMotor) + transform.up * blf);
        // Debug.DrawLine(transform.TransformPoint(brMotor), transform.TransformPoint(brMotor) + transform.up * brf);
    }

    public void Move(InputAction.CallbackContext context) {
        leftJoystick = context.ReadValue<Vector2>();
    }

    public void Look(InputAction.CallbackContext context) {
        rightJoystick = context.ReadValue<Vector2>();
    }
}
