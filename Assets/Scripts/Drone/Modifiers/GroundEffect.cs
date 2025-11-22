using UnityEngine;

[CreateAssetMenu(fileName = "GroundEffect", menuName = "Drone/Modifiers/Post-Processors/Ground Effect")]
public class GroundEffect : RotorForcePostProcessor
{
    [Tooltip("The radius of the rotors, used in the ground effect calculation.")]
    public float rotorRadius = 0.2f;

    public override void PostProcess(Drone drone, ref float[] rotorForces)
    {
        for (int i = 0; i < 4; i++)
        {
            rotorForces[i] *= 1f + 6f * Mathf.Exp(-Mathf.Max(drone.transform.position.y, 0) * 5f / rotorRadius);
        }
    }
}
