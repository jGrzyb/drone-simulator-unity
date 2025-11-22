using UnityEngine;

[CreateAssetMenu(fileName = "TransformTiltEstimator", menuName = "Drone/Estimators/Direct Transform")]
public class TransformTiltEstimator : TiltEstimator
{
    public override void Initialize(Drone drone)
    {
    }

    public override void Estimate(Drone drone)
    {

        Vector3 localAngularVelocity = drone.transform.InverseTransformDirection(drone.Rb.angularVelocity);
        RollRate = -localAngularVelocity.z * Mathf.Rad2Deg;
        PitchRate = localAngularVelocity.x * Mathf.Rad2Deg;

        Vector3 localGravity = drone.transform.InverseTransformDirection(Physics.gravity.normalized);
        RollAngle = Mathf.Atan2(localGravity.x, -localGravity.y) * Mathf.Rad2Deg;
        PitchAngle = Mathf.Atan2(localGravity.z, -localGravity.y) * Mathf.Rad2Deg;
    }
}
