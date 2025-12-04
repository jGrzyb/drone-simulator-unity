using UnityEngine;

[CreateAssetMenu(fileName = "Tilt Target Modifier", menuName = "Drone/Target Modifiers/Tilt Target Modifier")]
public class TiltTargetModifier : ITargetModifier {
    public override DesiredTilt GetDesiredTilt() {
            float desiredRoll = -InputManager.I.rightJoystick.x;
            float desiredPitch = InputManager.I.rightJoystick.y;
        return new DesiredTilt(desiredRoll, desiredPitch);
    }
}

