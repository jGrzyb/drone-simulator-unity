using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public class UIManager : MonoBehaviour
{
    public static UIManager I { get; private set; }

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

        fluentToggle.onValueChanged.AddListener(OnFluentToggleChanged);
        groundEffectToggle.onValueChanged.AddListener(OnGroundEffectToggleChanged);
        velocityDependentToggle.onValueChanged.AddListener(OnVelocityDependentToggleChanged);
    }

    void Start()
    {
        OnTiltEstimatorDropdownChanged(tiltEstimatorDropdown.value);
        OnTargetModifierDropdownChanged(targetModifierDropdown.value);
        OnControlAllocatorDropdownChanged(controlAllocatorDropdown.value);
        OnFluentToggleChanged(fluentToggle.isOn);
        OnGroundEffectToggleChanged(groundEffectToggle.isOn);
        OnVelocityDependentToggleChanged(velocityDependentToggle.isOn);
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
}
