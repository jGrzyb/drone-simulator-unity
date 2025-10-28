using UnityEngine;

public class CameraRotate : MonoBehaviour
{
    Drone drone;
    void Awake()
    {
        drone = FindFirstObjectByType<Drone>();
    }

    void Update()
    {
        transform.LookAt(drone.transform);        
    }
}
