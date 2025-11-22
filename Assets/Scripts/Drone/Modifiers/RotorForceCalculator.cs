using UnityEngine;

public abstract class RotorForceCalculator : ScriptableObject
{
    public abstract float[] Calculate(Drone drone, float upwardForce, float rollControl, float pitchControl, float yawControl);
}
