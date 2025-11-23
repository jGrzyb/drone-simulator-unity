using UnityEngine;
using UnityEngine.InputSystem;

public class CameraRotate : MonoBehaviour
{
    public enum CameraMode
    {
        Follow,
        FPV,
        Ground
    }

    [Header("Settings")]
    [SerializeField] private CameraMode currentMode = CameraMode.Follow;

    [Header("Follow Mode")]
    [SerializeField] private Vector3 followOffset = new Vector3(0, 2, -5);
    [SerializeField] private float followPositionSmoothTime = 0.1f;
    [SerializeField] private float followRotationSmoothSpeed = 5f;

    private Vector3 followVelocity = Vector3.zero;

    [Header("FPV Mode")]
    [SerializeField] private Vector3 fpvOffset = new Vector3(0, 0, 0.2f);

    [Header("Ground Mode")]
    [SerializeField] private Vector3 groundPosition = new Vector3(0, 1, -15);
    [SerializeField] private float groundZoomMultiplier = 1f;
    [SerializeField] private float minFovGround = 10f;
    [SerializeField] private float maxFovGround = 60f;

    private Drone drone;
    private Camera mainCamera;
    
    void Awake()
    {
        mainCamera = Camera.main;
        drone = FindFirstObjectByType<Drone>();



        if (GetComponent<SpringJoint>() != null)
        {
            Destroy(GetComponent<SpringJoint>());
        }
        if (GetComponent<Rigidbody>() != null)
        {
            Destroy(GetComponent<Rigidbody>());
        }
    }



    public void SwitchMode()
    {
        currentMode = (CameraMode)(((int)currentMode + 1) % System.Enum.GetValues(typeof(CameraMode)).Length);
    }

    public void ResetState()
    {
        followVelocity = Vector3.zero;
    }

    void LateUpdate()
    {
        if (drone == null) return;

        switch (currentMode)
        {
            case CameraMode.Follow:
                HandleFollowMode();
                break;
            case CameraMode.FPV:
                HandleFPVMode();
                break;
            case CameraMode.Ground:
                HandleGroundMode();
                break;
        }

        UpdateFOV();
    }

    void HandleFollowMode()
    {
        Quaternion yawRotation = Quaternion.Euler(0, drone.transform.eulerAngles.y, 0);
        Vector3 desiredPosition = drone.transform.position + (yawRotation * followOffset);

        transform.position = Vector3.SmoothDamp(transform.position, desiredPosition, ref followVelocity, followPositionSmoothTime);
        
        Quaternion desiredRotation = Quaternion.LookRotation(drone.transform.position - transform.position);
        transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, followRotationSmoothSpeed * Time.deltaTime);
    }

    void HandleFPVMode()
    {
        transform.position = drone.transform.TransformPoint(fpvOffset);
        transform.rotation = drone.transform.rotation;
    }

    void HandleGroundMode()
    {
        transform.position = groundPosition;
        transform.LookAt(drone.transform);
    }

    void UpdateFOV()
    {
        if (drone.Rb == null) return;

        float targetFov;
        float currentVelocityFovFactor = drone.Rb.linearVelocity.magnitude * 2f;

        switch (currentMode)
        {
            case CameraMode.Follow:
            case CameraMode.FPV:
                targetFov = 60f + currentVelocityFovFactor;
                mainCamera.fieldOfView = Mathf.Lerp(mainCamera.fieldOfView, Mathf.Clamp(targetFov, 60f, 100f), Time.deltaTime * 5f);
                break;
            case CameraMode.Ground:
                float distanceToDrone = Vector3.Distance(transform.position, drone.transform.position);
                targetFov = maxFovGround - (distanceToDrone * groundZoomMultiplier);
                mainCamera.fieldOfView = Mathf.Lerp(mainCamera.fieldOfView, Mathf.Clamp(targetFov, minFovGround, maxFovGround), Time.deltaTime * 5f);
                break;
        }
    }
}