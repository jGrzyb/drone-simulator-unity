using UnityEngine;

public abstract class TargetModifier : ScriptableObject
{
    public abstract void Modify(Drone drone, ref float desiredRoll, ref float desiredPitch);

    public virtual void ResetState() { }
}
