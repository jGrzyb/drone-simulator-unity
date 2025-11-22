using UnityEngine;

[CreateAssetMenu(fileName = "DefaultTargetModifier", menuName = "Drone/Modifiers/Target/Default")]
public class DefaultTargetModifier : TargetModifier
{
    [Header("Tuning")]
    [Tooltip("The maximum tilt angle of the drone in degrees.")]
    public float maxTiltAngle = 30f;

    public override void Modify(Drone drone, ref float desiredRoll, ref float desiredPitch)
    {
        desiredRoll = -drone.RightJoystick.x * maxTiltAngle * Mathf.Deg2Rad;
        desiredPitch = drone.RightJoystick.y * maxTiltAngle * Mathf.Deg2Rad;
    }
}
