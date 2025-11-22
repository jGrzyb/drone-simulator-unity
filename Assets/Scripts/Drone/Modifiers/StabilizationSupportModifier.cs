using UnityEngine;

[CreateAssetMenu(fileName = "StabilizationSupportModifier", menuName = "Drone/Modifiers/Target/Stabilization Support")]
public class StabilizationSupportModifier : TargetModifier
{
    [Header("Tuning")]
    [Tooltip("The maximum tilt angle of the drone in degrees.")]
    public float maxTiltAngle = 30f;

    [Tooltip("How strongly the drone tries to counteract movement when joystick is neutral.")]
    public float supportGain = 0.5f;

    public override void Modify(Drone drone, ref float desiredRoll, ref float desiredPitch)
    {
        if (drone.RightJoystick.magnitude < 0.1f)
        {
            float desiredRightVelocity = -drone.LocalLinearVelocity.x;
            float desiredForwardVelocity = -drone.LocalLinearVelocity.z;

            desiredRoll = Mathf.Atan(-desiredRightVelocity * supportGain) / (Mathf.PI / 2) * maxTiltAngle * Mathf.Deg2Rad;
            desiredPitch = Mathf.Atan(desiredForwardVelocity * supportGain) / (Mathf.PI / 2) * maxTiltAngle * Mathf.Deg2Rad;
        }
        else
        {
            desiredRoll = -drone.RightJoystick.x * maxTiltAngle * Mathf.Deg2Rad;
            desiredPitch = drone.RightJoystick.y * maxTiltAngle * Mathf.Deg2Rad;
        }
    }
}
