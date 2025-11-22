using UnityEngine;

[CreateAssetMenu(fileName = "FluentRotorSmoother", menuName = "Drone/Modifiers/Fluent Rotor Smoother")]
public class FluentRotorSmoother : ScriptableObject
{
    [Tooltip("The maximum change in rotor force per FixedUpdate step.")]
    public float maxRotorDelta = 0.5f;

    public float[] Smooth(Drone drone, float[] newForces)
    {
        for (int i = 0; i < 4; i++)
        {
            drone.RotorForcesArray[i] = Mathf.MoveTowards(drone.RotorForcesArray[i], newForces[i], maxRotorDelta);
        }
        return drone.RotorForcesArray;
    }
}
