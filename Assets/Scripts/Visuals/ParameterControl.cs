using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using TMPro;
using System;

public class ParameterControl : MonoBehaviour
{
    [Header("UI References")]
    [SerializeField] private TMP_Text parameterLabel;
    [SerializeField] private Slider parameterSlider;
    [SerializeField] private TMP_InputField parameterInput;

    public UnityEvent<float> OnValueChanged = new UnityEvent<float>();

    private bool isInitialized = false;

    private void Awake()
    {
        parameterSlider.onValueChanged.AddListener(OnSliderChanged);
        parameterInput.onEndEdit.AddListener(OnInputChanged);
    }

    public void Initialize(string label, float min, float max, float currentValue, Action<float> setter)
    {
        parameterLabel.text = label;
        parameterSlider.minValue = min;
        parameterSlider.maxValue = max;
        
        SetValue(currentValue);

        OnValueChanged.RemoveAllListeners();
        OnValueChanged.AddListener(value => setter(value));
        isInitialized = true;
    }

    public void SetValue(float value)
    {
        value = Mathf.Clamp(value, parameterSlider.minValue, parameterSlider.maxValue);
        parameterSlider.value = value;
        parameterInput.text = value.ToString("F3");
    }

    private void OnSliderChanged(float value)
    {
        if (!isInitialized) return;
        parameterInput.text = value.ToString("F3");
        OnValueChanged.Invoke(value);
    }

    private void OnInputChanged(string text)
    {
        if (!isInitialized) return;
        if (float.TryParse(text, out float value))
        {
            value = Mathf.Clamp(value, parameterSlider.minValue, parameterSlider.maxValue);
            parameterSlider.value = value; 
        }
        else
        {
            parameterInput.text = parameterSlider.value.ToString("F3");
        }
    }

    public void SetInteractable(bool interactable)
    {
        parameterLabel.color = interactable ? Color.white : Color.gray;
        parameterSlider.interactable = interactable;
        parameterInput.interactable = interactable;
    }
}
