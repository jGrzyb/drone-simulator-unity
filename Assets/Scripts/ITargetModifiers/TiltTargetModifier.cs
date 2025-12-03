using UnityEngine;

[CreateAssetMenu(fileName = "Tilt Target Modifier", menuName = "Drone/Target Modifiers/Tilt Target Modifier")]
public class TiltTargetModifier : ITargetModifier {
    public override DesiredTilt GetDesiredTilt() {
            float desiredRoll = -drone.rightJoystick.x * maxTiltAngle * Mathf.Deg2Rad;
            float desiredPitch = drone.rightJoystick.y * maxTiltAngle * Mathf.Deg2Rad;
        return new DesiredTilt(desiredRoll, desiredPitch);
    }
}

