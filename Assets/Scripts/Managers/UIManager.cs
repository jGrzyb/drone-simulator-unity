using System.Collections.Generic;
using System.Linq;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public class UIManager : MonoBehaviour
{
    public static UIManager I { get; private set; }

    [SerializeField] public FieldSlider maxTiltAngleSlider;
    [SerializeField] public FieldSlider tiltGainSlider;
    [SerializeField] public FieldSlider tiltDampingSlider;
    [SerializeField] public FieldSlider maxYawRateSlider;
    [SerializeField] public FieldSlider yawGainSlider;
    [SerializeField] public FieldSlider maxVerticalVelocitySlider;
    [SerializeField] public FieldSlider verticalGainSlider;
    [SerializeField] public FieldSlider airResistanceSlider;
    [SerializeField] public FieldSlider rotorDistanceSlider;
    [SerializeField] public Toggle isRotorInFrontToggle;

    [SerializeField] TMP_Dropdown tiltEstimatorDropdown;
    [SerializeField] public FieldSlider accelerometerNoiseSlider;
    [SerializeField] public FieldSlider gyroscopeNoiseSlider;
    [SerializeField] public FieldSlider gyroBiasDriftSlider;
    [HideInInspector] public UnityEvent onKalmanSelected = new UnityEvent();
    [HideInInspector] public UnityEvent onTransformSelected = new UnityEvent();
    private Dictionary<string, UnityEvent> estimatorEvents;

    [SerializeField] TMP_Dropdown targetModifierDropdown;
    [SerializeField] public FieldSlider supportGainSlider;
    [HideInInspector] public UnityEvent onSupportTargetSelected = new UnityEvent();
    [HideInInspector] public UnityEvent onTiltTargetSelected = new UnityEvent();
    private Dictionary<string, UnityEvent> targetModifierEvents;

    [SerializeField] TMP_Dropdown controlAllocatorDropdown;
    [SerializeField] public FieldSlider minRotorValueSlider;
    [SerializeField] public FieldSlider maxRotorValueSlider;
    [HideInInspector] public UnityEvent onConstrainedSelected = new UnityEvent();
    [HideInInspector] public UnityEvent onUnconstrainedSelected = new UnityEvent();
    private Dictionary<string, UnityEvent> controlAllocatorEvents;

    [SerializeField] Toggle fluentToggle;
    [SerializeField] public FieldSlider maxStepSizeSlider;
    [HideInInspector] public UnityEvent onFluentOn = new UnityEvent();
    [HideInInspector] public UnityEvent onFluentOff = new UnityEvent();

    [SerializeField] Toggle groundEffectToggle;
    [SerializeField] public FieldSlider rotorRadiusSlider;
    [HideInInspector] public UnityEvent onGroundOn = new UnityEvent();
    [HideInInspector] public UnityEvent onGroundOff = new UnityEvent();

    [SerializeField] Toggle velocityDependentToggle;
    [HideInInspector] public UnityEvent onVelDependentOn = new UnityEvent();
    [HideInInspector] public UnityEvent onVelDependentOff = new UnityEvent();

    [SerializeField] TMP_Dropdown environmentDropdown;
    [SerializeField] public FieldSlider droneCountX;
    [SerializeField] public FieldSlider droneCountY;
    [SerializeField] public FieldSlider droneCountZ;

    private readonly Dictionary<string, SimulationManager.SimulationEnv> envOptions = new()
    {
        { "Plane", SimulationManager.SimulationEnv.Plane },
        { "City", SimulationManager.SimulationEnv.City },
        { "Mountains", SimulationManager.SimulationEnv.Mountains }
    };

    void Awake()
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

        estimatorEvents = new Dictionary<string, UnityEvent>
        {
            { "KalmanTiltEstimator", onKalmanSelected },
            { "TransformTiltEstimator", onTransformSelected }
        };
        tiltEstimatorDropdown.ClearOptions();
        tiltEstimatorDropdown.AddOptions(estimatorEvents.Keys.ToList());
        tiltEstimatorDropdown.onValueChanged.AddListener(OnTiltEstimatorDropdownChanged);

        targetModifierEvents = new Dictionary<string, UnityEvent>
        {
            { "SupportTargetModifier", onSupportTargetSelected },
            { "TiltTargetModifier", onTiltTargetSelected }
        };
        targetModifierDropdown.ClearOptions();
        targetModifierDropdown.AddOptions(targetModifierEvents.Keys.ToList());
        targetModifierDropdown.onValueChanged.AddListener(OnTargetModifierDropdownChanged);

        controlAllocatorEvents = new Dictionary<string, UnityEvent>
        {
            { "ConstrainedControlAllocator", onConstrainedSelected },
            { "UnconstrainedControlAllocator", onUnconstrainedSelected }
        };
        controlAllocatorDropdown.ClearOptions();
        controlAllocatorDropdown.AddOptions(controlAllocatorEvents.Keys.ToList());
        controlAllocatorDropdown.onValueChanged.AddListener(OnControlAllocatorDropdownChanged);

        environmentDropdown.ClearOptions();
        environmentDropdown.AddOptions(envOptions.Keys.ToList());

        fluentToggle.onValueChanged.AddListener(OnFluentToggleChanged);
        groundEffectToggle.onValueChanged.AddListener(OnGroundEffectToggleChanged);
        velocityDependentToggle.onValueChanged.AddListener(OnVelocityDependentToggleChanged);
    }

    void Start() {
        RefreshAllParameters();
    }

    public void RefreshAllParameters() {
        maxTiltAngleSlider.onValueChanged.Invoke(maxTiltAngleSlider.GetCurrentValue());
        tiltGainSlider.onValueChanged.Invoke(tiltGainSlider.GetCurrentValue());
        tiltDampingSlider.onValueChanged.Invoke(tiltDampingSlider.GetCurrentValue());
        maxYawRateSlider.onValueChanged.Invoke(maxYawRateSlider.GetCurrentValue());
        yawGainSlider.onValueChanged.Invoke(yawGainSlider.GetCurrentValue());
        airResistanceSlider.onValueChanged.Invoke(airResistanceSlider.GetCurrentValue());
        rotorDistanceSlider.onValueChanged.Invoke(rotorDistanceSlider.GetCurrentValue());
        isRotorInFrontToggle.onValueChanged.Invoke(isRotorInFrontToggle.isOn);

        OnTiltEstimatorDropdownChanged(tiltEstimatorDropdown.value);
        OnTargetModifierDropdownChanged(targetModifierDropdown.value);
        OnControlAllocatorDropdownChanged(controlAllocatorDropdown.value);
        OnFluentToggleChanged(fluentToggle.isOn);
        OnGroundEffectToggleChanged(groundEffectToggle.isOn);
        OnVelocityDependentToggleChanged(velocityDependentToggle.isOn);
    }

    public void GetSimulationParameters(out int droneCountX, out int droneCountY, out int droneCountZ, out SimulationManager.SimulationEnv env)
    {
        droneCountX = (int)this.droneCountX.GetCurrentValue();
        droneCountY = (int)this.droneCountY.GetCurrentValue();
        droneCountZ = (int)this.droneCountZ.GetCurrentValue();
        env = envOptions[environmentDropdown.options[environmentDropdown.value].text];
    }

    private void OnTiltEstimatorDropdownChanged(int index)
    {
        estimatorEvents[tiltEstimatorDropdown.options[index].text]?.Invoke();
    }

    private void OnTargetModifierDropdownChanged(int index)
    {
        targetModifierEvents[targetModifierDropdown.options[index].text]?.Invoke();
    }

    private void OnControlAllocatorDropdownChanged(int index)
    {
        controlAllocatorEvents[controlAllocatorDropdown.options[index].text]?.Invoke();
    }

    private void OnFluentToggleChanged(bool isOn)
    {
        if (isOn) {
            onFluentOn.Invoke();
        }
        else {
            onFluentOff.Invoke();
        }
    }

    private void OnGroundEffectToggleChanged(bool isOn)
    {
        if (isOn) {
            onGroundOn.Invoke();
        }
        else {
            onGroundOff.Invoke();
        }
    }

    private void OnVelocityDependentToggleChanged(bool isOn)
    {
        if (isOn) {
            onVelDependentOn.Invoke();
        }
        else {
            onVelDependentOff.Invoke();
        }
    }

    public void ToggleUIVisibility()
    {
        Canvas canvas = GetComponent<Canvas>();
        canvas.enabled = !canvas.enabled;
    }
}
