using UnityEngine;

public abstract class ITiltEstimator : ScriptableObject
{
    protected Transform transform;
    protected Rigidbody rb;
    
    public virtual void Initialize(Drone drone) {
        transform = drone.transform;
        rb = drone.GetComponent<Rigidbody>();
    }
    public abstract void UpdateFilter();

    public abstract float GetRollAngle();
    public abstract float GetPitchAngle();
    public abstract float GetRollRate();
    public abstract float GetPitchRate();
}
