using UnityEngine;

public class CameraRotate : MonoBehaviour
{
    Drone drone;
    void Awake()
    {
        drone = FindFirstObjectByType<Drone>();
        GetComponent<SpringJoint>().connectedBody = drone.GetComponent<Rigidbody>();
    }

    void Update()
    {
        transform.LookAt(drone.transform);        
    }
}
