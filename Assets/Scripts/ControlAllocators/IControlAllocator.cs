using UnityEngine;

public abstract class IControlAllocator : ScriptableObject
{
    public abstract float[] Allocate(float upwardForce, float rollControl, float pitchControl, float yawControl, double[,] controlToRotorMatrix);
}