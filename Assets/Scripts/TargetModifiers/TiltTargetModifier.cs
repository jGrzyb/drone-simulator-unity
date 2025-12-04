using UnityEngine;

[CreateAssetMenu(fileName = "Tilt Target Modifier", menuName = "Drone/Target Modifiers/Tilt Target Modifier")]
public class TiltTargetModifier : ITargetModifier {
    public override DesiredTilt GetDesiredTilt() {
            float desiredRoll = -InputManager.I.rightJoystick.x * maxTiltAngle * Mathf.Deg2Rad;
            float desiredPitch = InputManager.I.rightJoystick.y * maxTiltAngle * Mathf.Deg2Rad;
        return new DesiredTilt(desiredRoll, desiredPitch);
    }
}

