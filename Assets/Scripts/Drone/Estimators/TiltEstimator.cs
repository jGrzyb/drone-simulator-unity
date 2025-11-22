using UnityEngine;

public abstract class TiltEstimator : ScriptableObject
{
    public float RollAngle { get; protected set; }
    public float RollRate { get; protected set; }
    public float PitchAngle { get; protected set; }
    public float PitchRate { get; protected set; }

    public abstract void Initialize(Drone drone);

    public abstract void Estimate(Drone drone);
}
