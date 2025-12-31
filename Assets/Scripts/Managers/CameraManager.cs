using UnityEngine;

public class CameraManager : MonoBehaviour {
    public static CameraManager I { get; private set; }
    [SerializeField] private Drone drone;
    [SerializeField] public float cameraSpeed = 2f;
    [SerializeField] public float minDistance = 3f;
    [SerializeField] public CameraMode cameraMode = CameraMode.Follow;

    private Vector3 initialPosition;
    private Quaternion initialRotation;

    private Vector3 dronePosition { get { return drone != null ? drone.transform.position : Vector3.zero; } }

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

    public void SetTarget(Drone drone) {
        this.drone = drone;
    }

    public enum CameraMode {
        Follow,
        Fixed,
        Ground,
        FPV
    }

    private void followView() {
        Vector3 aboveDronePos = dronePosition + Vector3.up * 2f;
        Vector3 targetToCamera = transform.position - aboveDronePos;
        Vector3 targetPosition = aboveDronePos + targetToCamera / targetToCamera.magnitude * minDistance;
        transform.position = Vector3.Lerp(transform.position, targetPosition, cameraSpeed * Time.deltaTime);
        transform.LookAt(dronePosition);
    }

    private void fixedView() {
        transform.LookAt(dronePosition);
    }

    private void groundView() {
        transform.position = new Vector3(0, 2, 0);
        transform.LookAt(dronePosition);
    }

    private void fpvView() {
        if (drone != null) {
            transform.position = dronePosition + drone.transform.forward * 0.6f;
            transform.rotation = drone.transform.rotation;
        }
    }

    public void ResetCameraPosition() {
        transform.position = initialPosition;
        transform.rotation = initialRotation;
    }
}
