using UnityEngine;

public abstract class IControlAllocator : ScriptableObject
{
    public virtual void Initialize() {}
    public abstract float[] Allocate(float upwardForce, float rollControl, float pitchControl, float yawControl, double[,] controlToRotorMatrix);
}