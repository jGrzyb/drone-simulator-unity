using UnityEngine;

public class CameraRotate : MonoBehaviour {
    [SerializeField] private Drone drone;
    [SerializeField] public float cameraSpeed = 2f;
    [SerializeField] public float minDistance = 3f;
    [SerializeField] public CameraMode cameraMode = CameraMode.Follow;

    void Awake() {
        if (drone == null) {
            drone = FindFirstObjectByType<Drone>();
        }
    }

    void LateUpdate() {
        switch (cameraMode) {
            case CameraMode.Follow:
                followView();
                break;
            case CameraMode.Fixed:
                fixedView();
                break;
            case CameraMode.Ground:
                groundView();
                break;
            case CameraMode.FPV:
                fpvView();
                break;
        }
    }

    public enum CameraMode {
        Follow,
        Fixed,
        Ground,
        FPV
    }

    private void followView() {
        Vector3 aboveDronePos = drone.transform.position + Vector3.up * 2f;
        Vector3 targetToCamera = transform.position - aboveDronePos;
        Vector3 targetPosition = aboveDronePos + targetToCamera / targetToCamera.magnitude * minDistance;
        transform.position = Vector3.Lerp(transform.position, targetPosition, cameraSpeed * Time.deltaTime);
        transform.LookAt(drone.transform);
    }

    private void fixedView() {
        transform.LookAt(drone.transform);
    }

    private void groundView() {
        transform.position = new Vector3(0, 2, 0);
        transform.LookAt(drone.transform);
    }

    private void fpvView() {
        transform.position = drone.transform.position + drone.transform.forward * 0.6f;
        transform.rotation = drone.transform.rotation;
    }
}
