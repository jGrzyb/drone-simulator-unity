using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.Events;

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
    }

    void Start()
    {
        OnTiltEstimatorDropdownChanged(tiltEstimatorDropdown.value);
        OnTargetModifierDropdownChanged(targetModifierDropdown.value);
    }

    private void OnTiltEstimatorDropdownChanged(int index)
    {
        estimatorEvents[tiltEstimatorDropdown.options[index].text]?.Invoke();
    }

    private void OnTargetModifierDropdownChanged(int index)
    {
        targetModifierEvents[targetModifierDropdown.options[index].text]?.Invoke();
    }
}
