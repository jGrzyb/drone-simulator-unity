using UnityEngine;

[RequireComponent(typeof(Drone))]
public class DroneRotorVisualizer : MonoBehaviour
{
    [SerializeField] private LineRenderer rotorLinePrefab;
    [SerializeField] private float lineScale = 5f;

    private Drone drone;
    private LineRenderer[] rotorLines;
    private bool isVisible = true;

    void Awake()
    {
        drone = GetComponent<Drone>();
        InputManager.I.onDroneFeaturesVisibilityToggle.AddListener(() => MakeVisible(!isVisible));
    }

    void Start()
    {
        rotorLines = new LineRenderer[4];
        for (int i = 0; i < 4; i++)
        {
            rotorLines[i] = Instantiate(rotorLinePrefab, drone.transform);
        }
    }

    void Update()
    {
        Vector3[] rotorPositions = drone.rotorPoses;
        for (int i = 0; i < 4; i++)
        {
            Vector3 worldPos = drone.transform.TransformPoint(rotorPositions[i]);
            rotorLines[i].SetPosition(0, worldPos);
            float thrust = drone.rotorForcesArray[i] * lineScale;
            rotorLines[i].SetPosition(1, worldPos + drone.transform.up * thrust * 0.1f);
        }
    }

    void MakeVisible(bool isVisible)
    {
        this.isVisible = isVisible;
        foreach (var line in rotorLines)
        {
            line.enabled = isVisible;
        }
        GetComponent<TrailRenderer>().enabled = isVisible;
    }
}