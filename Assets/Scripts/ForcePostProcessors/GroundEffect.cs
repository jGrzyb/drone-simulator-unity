using UnityEngine;

[CreateAssetMenu(fileName = "Ground Effect", menuName = "Drone/Ground Effect")]
public class GroundEffect : ScriptableObject
{
    [SerializeField] public float rotorRadius = 0.2f;

    public void Initialize() {
        UIManager.I.rotorRadiusSlider.onValueChanged.AddListener(value => rotorRadius = value);
    }

    public void ModifyAppliedForces(ref float[] appliedForces, Vector3[] rotorPositions, Transform droneTransform) {
        for (int i = 0; i < 4; i++) {
            bool isHit = Physics.Raycast(droneTransform.TransformPoint(rotorPositions[i]), -droneTransform.up, out RaycastHit hitInfo, Mathf.Infinity);
            if (isHit) {
                float height = hitInfo.distance;
                appliedForces[i] *= 1f + 6f * Mathf.Exp(-Mathf.Max(height, 0) * 5f / rotorRadius);
            }
        }
    }
}