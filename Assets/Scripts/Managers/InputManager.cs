using System.Linq;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;

[RequireComponent(typeof(PlayerInput))]
public class InputManager : MonoBehaviour
{
    public static InputManager I { get; private set; }
    public Vector2 leftJoystick { get; private set; }
    public Vector2 rightJoystick { get; private set; }
    public UnityEvent onDroneFeaturesVisibilityToggle = new UnityEvent();

    private void Awake()
    {
        if (I == null)
        {
            I = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }

    public void GetLeftStick(InputAction.CallbackContext context) {
        leftJoystick = context.ReadValue<Vector2>();
    }

    public void GetRightStick(InputAction.CallbackContext context) {
        rightJoystick = context.ReadValue<Vector2>();
    }

    public void GetCameraSwitch(InputAction.CallbackContext context) {
        if (context.performed) {
            CameraRotate.I.SwitchCameraMode();
        }
    }

    public void GetReset(InputAction.CallbackContext context) {
        if (context.performed) {
            FindObjectsByType<Drone>(FindObjectsSortMode.None).ToList().ForEach(drone => drone.ResetDronePosition());
            CameraRotate.I.ResetCameraPosition();
        }
    }

    public void GetUIShowHide(InputAction.CallbackContext context) {
        if (context.performed) {
            UIManager.I.ToggleUIVisibility();
        }
    }

    public void GetPause(InputAction.CallbackContext context) {
        if (context.performed) {
            Time.timeScale = Time.timeScale == 0 ? 1 : 0;
        }
    }

    public void GetDroneFeaturesVisibilityToggle(InputAction.CallbackContext context) {
        if (context.performed) {
            onDroneFeaturesVisibilityToggle.Invoke();
        }
    }

    public void MeasurePerformance(InputAction.CallbackContext context) {
        if (context.performed) {
            FindFirstObjectByType<PerformanceMonitor>()?.Measure();
        }
    }
}