using UnityEngine;

public class CameraRotate : MonoBehaviour {
    public static CameraRotate I { get; private set; }
    [SerializeField] private Drone drone;
    [SerializeField] public float cameraSpeed = 2f;
    [SerializeField] public float minDistance = 3f;
    [SerializeField] public CameraMode cameraMode = CameraMode.Follow;

    private Vector3 initialPosition;
    private Quaternion initialRotation;

    void Awake() {
        if (I == null) {
            I = this;
            DontDestroyOnLoad(gameObject);
        } else {
            Destroy(gameObject);
        }
        if (drone == null) {
            drone = FindFirstObjectByType<Drone>();
        }
        initialPosition = transform.position;
        initialRotation = transform.rotation;
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

    public void SwitchCameraMode() {
        cameraMode = (CameraMode)(((int)cameraMode + 1) % System.Enum.GetNames(typeof(CameraMode)).Length);
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

    public void ResetCameraPosition() {
        transform.position = initialPosition;
        transform.rotation = initialRotation;
    }
}
