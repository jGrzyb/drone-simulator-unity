using UnityEngine;

[CreateAssetMenu(fileName = "Support Target Modifier", menuName = "Drone/Target Modifiers/Support Target Modifier")]
public class SupportTargetModifier : ITargetModifier {
    [SerializeField] public float supportGain = 0.5f;
    public override DesiredTilt GetDesiredTilt()
    {
        float desiredRoll;
        float desiredPitch;
        Vector3 localVelocity = Quaternion.Inverse(Quaternion.Euler(0, transform.eulerAngles.y, 0)) * rb.linearVelocity;
        if (InputManager.I.rightJoystick.magnitude < 0.1f) {
            float desiredRightVelocity = -localVelocity.x;
            float desiredForwardVelocity = -localVelocity.z;

            desiredRoll = Mathf.Atan(-desiredRightVelocity * supportGain) / (Mathf.PI / 2);
            desiredPitch = Mathf.Atan(desiredForwardVelocity * supportGain) / (Mathf.PI / 2);
        } else {
            desiredRoll = -InputManager.I.rightJoystick.x;
            desiredPitch = InputManager.I.rightJoystick.y;
        }
        return new DesiredTilt(desiredRoll, desiredPitch);
    }
}

