using UnityEngine;

[CreateAssetMenu(fileName = "VelocityThrustInfluence", menuName = "Drone/Modifiers/Post-Processors/Velocity Thrust Influence")]
public class VelocityThrustInfluence : RotorForcePostProcessor
{
    public override void PostProcess(Drone drone, ref float[] rotorForces)
    {
        for (int i = 0; i < 4; i++)
        {
            Vector3 pointVelocity = drone.Rb.GetPointVelocity(drone.transform.TransformPoint(drone.RotorPoses[i]));
            Vector3 localPointVelocity = drone.transform.InverseTransformDirection(pointVelocity);
            float rotorLocalVerticalVelocity = localPointVelocity.y;
            rotorForces[i] *= 1f - rotorLocalVerticalVelocity / Mathf.Max(Mathf.Sqrt(Mathf.Abs(rotorForces[i])), 0.1f);
        }
    }
}
