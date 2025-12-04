using UnityEngine;

public abstract class ITargetModifier : ScriptableObject
{
    protected Transform transform;
    protected Rigidbody rb;
    
    public virtual void Initialize(Drone drone)
    {
        transform = drone.transform;
        rb = drone.GetComponent<Rigidbody>();
    }

    public abstract DesiredTilt GetDesiredTilt();
}