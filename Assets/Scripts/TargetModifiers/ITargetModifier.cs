using UnityEngine;

public abstract class ITargetModifier : ScriptableObject
{
    public float maxTiltAngle = 30f;
    public float supportGain = 0.5f;
    protected Transform transform;
    protected Rigidbody rb;
    
    public virtual void Initialize(Drone drone)
    {
        transform = drone.transform;
        rb = drone.GetComponent<Rigidbody>();
    }

    public abstract DesiredTilt GetDesiredTilt();
}