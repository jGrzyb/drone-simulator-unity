using UnityEngine;

/// <summary>
/// Abstract base class for a strategy that estimates the drone's tilt and angular rates.
/// </summary>
public abstract class TiltEstimator : ScriptableObject
{
    public float RollAngle { get; protected set; }
    public float RollRate { get; protected set; }
    public float PitchAngle { get; protected set; }
    public float PitchRate { get; protected set; }

    /// <summary>
    /// Called once to initialize the estimator.
    /// </summary>
    public abstract void Initialize(Drone drone);

    /// <summary>
    /// Called every FixedUpdate to perform the estimation.
    /// </summary>
    public abstract void Estimate(Drone drone);
}
