using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public class FieldSlider : MonoBehaviour
{
    [SerializeField] private TMP_InputField inputField;
    [SerializeField] private Slider slider;
    [SerializeField] private TMP_Text label;
    [SerializeField] private float minValue = 0f;
    [SerializeField] private float maxValue = 1f;
    [SerializeField] private float initialValue = 0.5f;
    [SerializeField] private string labelText = "Field Slider";
    [HideInInspector]
    public UnityEvent<float> onValueChanged = new UnityEvent<float>();

    private void Awake()
    {
        label.text = labelText;
        inputField.onEndEdit.AddListener(OnInputFieldChanged);
        slider.onValueChanged.AddListener(OnSliderChanged);
        onValueChanged.AddListener(UpdateComponents);
        slider.minValue = minValue;
        slider.maxValue = maxValue;
    }

    void Start()
    {
        onValueChanged.Invoke(initialValue);
    }

    private void OnInputFieldChanged(string value)
    {
        if (float.TryParse(value, out float result))
        {
            result = Mathf.Clamp(result, minValue, maxValue);
            onValueChanged.Invoke(result);
        }
    }

    private void OnSliderChanged(float value)
    {
        onValueChanged.Invoke(value);
    }

    private void UpdateComponents(float value)
    {
        slider.value = value;
        inputField.text = value.ToString("F4");
    }

    public float GetInitialValue()
    {
        return initialValue;
    }

    public float GetCurrentValue()
    {
        return slider.value;
    }
}