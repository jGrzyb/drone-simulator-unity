using UnityEngine;

public class CameraRotate : MonoBehaviour
{
    private Drone drone;
    private Rigidbody rb;
    private Camera mainCamera;
    void Awake()
    {
        mainCamera = Camera.main;
        rb = GetComponent<Rigidbody>();
        drone = FindFirstObjectByType<Drone>();
        GetComponent<SpringJoint>().connectedBody = drone.GetComponent<Rigidbody>();
    }

    void Update()
    {
        transform.LookAt(drone.transform);
        mainCamera.fieldOfView = Mathf.Clamp(60f + rb.linearVelocity.magnitude, 60f, 100f);
    }
}
