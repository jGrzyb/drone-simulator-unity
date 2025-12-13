using UnityEngine;

[CreateAssetMenu(fileName = "Ground Effect", menuName = "Drone/Ground Effect")]
public class GroundEffect : ScriptableObject
{
    [SerializeField] public float rotorRadius = 0.2f;

    public void Initialize() {
        UIManager.I.rotorRadiusSlider.onValueChanged.AddListener(value => rotorRadius = value);
    }

    public void ModifyAppliedForces(ref float[] appliedForces, Vector3 position) {
        for (int i = 0; i < 4; i++) {
            appliedForces[i] *= 1f + 6f * Mathf.Exp(-Mathf.Max(position.y, 0) * 5f / rotorRadius);
        }
    }
}