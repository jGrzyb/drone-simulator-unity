using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;

[RequireComponent(typeof(PlayerInput))]
public class InputManager : MonoBehaviour
{
    public static InputManager I { get; private set; }
    public Vector2 leftJoystick { get; private set; }
    public Vector2 rightJoystick { get; private set; }

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
}