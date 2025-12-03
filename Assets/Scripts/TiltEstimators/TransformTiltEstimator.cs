

using UnityEngine;

[CreateAssetMenu(fileName = "Transform Tilt Estimator", menuName = "Drone/Tilt Estimators/Transform Tilt Estimator")]
public class TransformTiltEstimator : ITiltEstimator
{
    public override void UpdateFilter()
    {
    }

    public override float GetRollAngle()
    {
        Vector3 localGravity = transform.InverseTransformDirection(Physics.gravity.normalized);
        return Mathf.Atan2(localGravity.x, -localGravity.y) * Mathf.Rad2Deg;
    }

    public override float GetPitchAngle()
    {
        Vector3 localGravity = transform.InverseTransformDirection(Physics.gravity.normalized);
        return Mathf.Atan2(localGravity.z, -localGravity.y) * Mathf.Rad2Deg;
    }

    public override float GetRollRate()
    {
        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
        return -localAngularVelocity.z * Mathf.Rad2Deg;
    }

    public override float GetPitchRate()
    {
        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
        return localAngularVelocity.x * Mathf.Rad2Deg;
    }
}
