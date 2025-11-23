using UnityEngine;

public class SceneManager : MonoBehaviour
{
    private Drone drone;
    private CameraRotate gameCamera;
    private DroneUIController uiController;

    private Vector3 initialDronePosition;
    private Quaternion initialDroneRotation;
    private Vector3 initialCameraPosition;
    private Quaternion initialCameraRotation;

    void Start()
    {
        drone = FindFirstObjectByType<Drone>();
        gameCamera = FindFirstObjectByType<CameraRotate>();
        uiController = FindFirstObjectByType<DroneUIController>();

        if (drone != null)
        {
            initialDronePosition = drone.transform.position;
            initialDroneRotation = drone.transform.rotation;
        }
        if (gameCamera != null)
        {
            initialCameraPosition = gameCamera.transform.position;
            initialCameraRotation = gameCamera.transform.rotation;
        }
        
        if (drone != null)
        {
            drone.OnRestart.AddListener(ResetScene);
            drone.OnSwitchCamera.AddListener(SwitchCamera);
            drone.OnToggleUI.AddListener(ToggleUI);
        }
    }

    void OnDestroy()
    {
        if (drone != null)
        {
            drone.OnRestart.RemoveListener(ResetScene);
            drone.OnSwitchCamera.RemoveListener(SwitchCamera);
            drone.OnToggleUI.RemoveListener(ToggleUI);
        }
    }

    private void ResetScene()
    {
        if (drone != null)
        {
            var rb = drone.Rb;
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.position = initialDronePosition;
                rb.rotation = initialDroneRotation;
            }
            drone.ResetState();
        }

        if (gameCamera != null)
        {
            gameCamera.transform.position = initialCameraPosition;
            gameCamera.transform.rotation = initialCameraRotation;
            gameCamera.ResetState();
        }
    }

    private void SwitchCamera()
    {
        if (gameCamera != null)
        {
            gameCamera.SwitchMode();
        }
    }

    private void ToggleUI()
    {
        if (uiController != null)
        {
            uiController.ToggleVisibility();
        }
    }
}
