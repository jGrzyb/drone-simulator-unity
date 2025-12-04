using UnityEngine;

[CreateAssetMenu(fileName = "Support Target Modifier", menuName = "Drone/Target Modifiers/Support Target Modifier")]
public class SupportTargetModifier : ITargetModifier {
    public override DesiredTilt GetDesiredTilt()
    {
        float desiredRoll;
        float desiredPitch;
        Vector3 localVelocity = Quaternion.Inverse(Quaternion.Euler(0, transform.eulerAngles.y, 0)) * rb.linearVelocity;
        if (InputManager.I.rightJoystick.magnitude < 0.1f) {
            float desiredRightVelocity = -localVelocity.x;
            float desiredForwardVelocity = -localVelocity.z;

            desiredRoll = Mathf.Atan(-desiredRightVelocity * supportGain) / (Mathf.PI / 2) * maxTiltAngle * Mathf.Deg2Rad;
            desiredPitch = Mathf.Atan(desiredForwardVelocity * supportGain) / (Mathf.PI / 2) * maxTiltAngle * Mathf.Deg2Rad;
        } else {
            desiredRoll = -InputManager.I.rightJoystick.x * maxTiltAngle * Mathf.Deg2Rad;
            desiredPitch = InputManager.I.rightJoystick.y * maxTiltAngle * Mathf.Deg2Rad;
        }
        return new DesiredTilt(desiredRoll, desiredPitch);
    }
}

