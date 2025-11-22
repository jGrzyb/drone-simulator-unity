using UnityEngine;

[RequireComponent(typeof(Drone))]
public class DroneVisualizer : MonoBehaviour
{
    [Header("References")]
    [Tooltip("The prefab for the line renderer used to show rotor forces. This prefab should have a LineRenderer component.")]
    [SerializeField] private LineRenderer rotorLinePrefab;

    [Header("Settings")]
    [Tooltip("A multiplier to control the visible length of the force lines.")]
    [SerializeField] private float forceLineScale = 0.5f;

    private LineRenderer[] rotorLines;
    private Drone drone;

    void Awake()
    {
        if (drone == null)
        {
            drone = GetComponent<Drone>();
        }

        if (rotorLinePrefab == null)
        {
            Debug.LogError("Rotor Line Prefab is not assigned in the DroneVisualizer. Disabling component.", this);
            this.enabled = false;
            return;
        }

        rotorLines = new LineRenderer[4];
        for (int i = 0; i < 4; i++)
        {
            rotorLines[i] = Instantiate(rotorLinePrefab, transform);
            rotorLines[i].gameObject.name = $"RotorLine_{i}";
        }
    }

    void Update()
    {
        if (drone == null || drone.RotorPoses == null || drone.RotorPoses.Length != 4)
        {
            return;
        }

        for (int i = 0; i < 4; i++)
        {
            if (rotorLines[i] != null)
            {
                Vector3 rotorPos = drone.transform.TransformPoint(drone.RotorPoses[i]);
                Vector3 forceVector = drone.transform.up * drone.RotorForcesArray[i] * forceLineScale;

                rotorLines[i].SetPosition(0, rotorPos);
                rotorLines[i].SetPosition(1, rotorPos + forceVector);
            }
        }
    }
}