using UnityEngine;

/// <summary>
/// An estimator that acts as a "perfect sensor", reading tilt and rate
/// directly from the drone's transform and rigidbody without noise or filtering.
/// </summary>
[CreateAssetMenu(fileName = "TransformTiltEstimator", menuName = "Drone/Estimators/Direct Transform")]
public class TransformTiltEstimator : TiltEstimator
{
    public override void Initialize(Drone drone)
    {
        // No initialization needed for this simple estimator.
    }

    public override void Estimate(Drone drone)
    {
        // --- Get "perfect" sensor readings without noise ---

        // Get rates directly from rigidbody angular velocity
        Vector3 localAngularVelocity = drone.transform.InverseTransformDirection(drone.Rb.angularVelocity);
        RollRate = -localAngularVelocity.z * Mathf.Rad2Deg;
        PitchRate = localAngularVelocity.x * Mathf.Rad2Deg;

        // Get angles directly from the local gravity vector (like a perfect accelerometer)
        Vector3 localGravity = drone.transform.InverseTransformDirection(Physics.gravity.normalized);
        RollAngle = Mathf.Atan2(localGravity.x, -localGravity.y) * Mathf.Rad2Deg;
        PitchAngle = Mathf.Atan2(localGravity.z, -localGravity.y) * Mathf.Rad2Deg;
    }
}
