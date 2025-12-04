using UnityEngine;

public abstract class ITargetModifier : ScriptableObject
{
    public float maxTiltAngle = 30f;
    public float supportGain = 0.5f;
    protected Transform transform;
    protected Rigidbody rb;
    protected Drone drone;
    
    public virtual void Initialize(Drone drone)
    {
        transform = drone.transform;
        rb = drone.GetComponent<Rigidbody>();
        this.drone = drone;
    }

    public abstract DesiredTilt GetDesiredTilt();
}