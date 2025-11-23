using UnityEngine;

public abstract class RotorForcePostProcessor : ScriptableObject
{
    public abstract void PostProcess(Drone drone, ref float[] rotorForces);

    public virtual void ResetState() { }
}
