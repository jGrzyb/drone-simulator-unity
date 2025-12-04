using UnityEngine;

[CreateAssetMenu(fileName = "Fluent Rotors", menuName = "Drone/Fluent Rotors")]
public class FluentRotors : ScriptableObject
{
    [SerializeField] public float maxRotorDelta = 0.5f;
    public void ModifyRotorForces(ref float[] currentRotorForces, float[] targetRotorForces)
    {
        for (int i = 0; i < 4; i++) {
            currentRotorForces[i] = Mathf.MoveTowards(currentRotorForces[i], targetRotorForces[i], maxRotorDelta);
        }
    }
}