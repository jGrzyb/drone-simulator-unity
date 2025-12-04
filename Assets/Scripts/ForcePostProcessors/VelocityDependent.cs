using UnityEngine;

[CreateAssetMenu(fileName = "Velocity Dependent", menuName = "Drone/Velocity Dependent")]
public class VelocityDependent : ScriptableObject
{
    public void ModifyAppliedForces(ref float[] appliedRotorForces, Rigidbody rb, Transform transform, Vector3[] rotorPoses)
    {
        for (int i = 0; i < 4; i++) {
            Vector3 pv = rb.GetPointVelocity(transform.TransformPoint(rotorPoses[i]));
            Vector3 lpv = transform.InverseTransformDirection(pv);
            // Debug.Log($"Rotor {i} local vertical velocity: {lpv}");
            float rotorLocalVerticalVelocity = lpv.y;
            appliedRotorForces[i] *= 1f - rotorLocalVerticalVelocity / Mathf.Max(Mathf.Sqrt(Mathf.Abs(appliedRotorForces[i])), 0.1f);
            Debug.DrawLine(transform.TransformPoint(rotorPoses[i]), transform.TransformPoint(rotorPoses[i]) + pv, Color.red);
        }
    }
}