using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using System;

public class DroneUIController : MonoBehaviour {
    [Header("Main Reference")]
    [Tooltip("The Drone instance this UI will control.")]
    [SerializeField] private Drone drone;

    [Header("Multi-Strategy Presets")]
    public List<TiltEstimator> availableTiltEstimators;
    public List<TargetModifier> availableTargetModifiers;
    public List<RotorForceCalculator> availableRotorCalculators;

    [Header("Single-Strategy Presets")]
    public FluentRotorSmoother fluentRotorPreset;
    public GroundEffect groundEffectPreset;
    public VelocityThrustInfluence velocityInfluencePreset;

    [Header("Strategy UI")]
    public TMP_Dropdown tiltEstimatorDropdown;
    public TMP_Dropdown targetModifierDropdown;
    public TMP_Dropdown rotorCalculatorDropdown;
    [Space]
    public Toggle fluentRotorToggle;
    public Toggle groundEffectToggle;
    public Toggle velocityInfluenceToggle;

    [Header("Parameter Controls")]
    [Tooltip("Assign all ParameterControl prefabs from your UI here.")]
    public List<ParameterControl> parameterControls;
    [Tooltip("Assign the 'Is Rotor In Front' toggle here.")]
    public Toggle isRotorInFrontToggle;

    private Dictionary<string, ParameterControl> controlDict = new Dictionary<string, ParameterControl>();

    #region Override Value Storage
    private float? accelNoiseOverride, gyroNoiseOverride, qAngleOverride, qRateOverride, rAngleOverride, rRateOverride;
    private float? maxTiltAngleOverride, supportGainOverride;
    private float? maxRotorForceOverride, minRotorForceOverride;
    private float? maxRotorDeltaOverride;
    private float? rotorRadiusOverride;
    #endregion

    void Awake() {
        drone = drone ?? FindFirstObjectByType<Drone>();
    }

    void Start() {
        if (drone == null) {
            Debug.LogError("Drone reference not set in DroneUIController.", this);
            return;
        }

        foreach (var control in parameterControls) {
            controlDict[control.name] = control;
        }

        SetupStrategyControls();
        SetupParameterControls();
        UpdateUIVisibility();
    }

    private void SetupStrategyControls() {
        SetupMultiStrategyControl(tiltEstimatorDropdown, availableTiltEstimators.Cast<ScriptableObject>().ToList(), drone.CurrentTiltEstimator, (p) => drone.SetTiltEstimator(p as TiltEstimator));
        SetupMultiStrategyControl(targetModifierDropdown, availableTargetModifiers.Cast<ScriptableObject>().ToList(), drone.CurrentTargetModifier, (p) => drone.SetTargetModifier(p as TargetModifier));
        SetupMultiStrategyControl(rotorCalculatorDropdown, availableRotorCalculators.Cast<ScriptableObject>().ToList(), drone.CurrentRotorForceCalculator, (p) => drone.SetRotorForceCalculator(p as RotorForceCalculator));

        SetupToggleStrategyControl(fluentRotorToggle, fluentRotorPreset, drone.CurrentFluentRotor, (p) => drone.SetFluentRotor(p));
        SetupToggleStrategyControl(groundEffectToggle, groundEffectPreset, drone.CurrentGroundEffect, (p) => drone.SetGroundEffect(p));
        SetupToggleStrategyControl(velocityInfluenceToggle, velocityInfluencePreset, drone.CurrentVelocityThrustInfluence, (p) => drone.SetVelocityThrustInfluence(p));
    }

    private void SetupParameterControls() {
        GetControl("TiltGain")?.Initialize("Tilt Gain", 0, 20, drone.tiltGain, v => drone.tiltGain = v);
        GetControl("TiltDamping")?.Initialize("Tilt Damping", 0, 10, drone.tiltDamping, v => drone.tiltDamping = v);
        GetControl("YawGain")?.Initialize("Yaw Gain", 0, 10, drone.yawGain, v => drone.yawGain = v);
        GetControl("MaxYawRate")?.Initialize("Max Yaw Rate", 0, 10, drone.maxYawRate, v => drone.maxYawRate = v);
        GetControl("MaxVerticalVelocity")?.Initialize("Max Vert. Velocity", 0, 50, drone.maxVerticalVelocity, v => drone.maxVerticalVelocity = v);
        GetControl("VerticalGain")?.Initialize("Vertical Gain", 0, 10, drone.verticalGain, v => drone.verticalGain = v);
        GetControl("ThrustMultiplier")?.Initialize("Thrust Multiplier", 0, 5, drone.thrustMultiplier, v => drone.thrustMultiplier = v);
        GetControl("DragMultiplier")?.Initialize("Drag Multiplier", 0, 1, drone.dragMultiplier, v => drone.dragMultiplier = v);
        GetControl("AirResistance")?.Initialize("Air Resistance", 0, 2, drone.airResistanceCoefficient, v => drone.airResistanceCoefficient = v);
        GetControl("RotorDistance")?.Initialize("Rotor Distance", 0.1f, 5, drone.rotorDistance, v => drone.rotorDistance = v);
        SetupLinkedToggle(isRotorInFrontToggle, drone.isRotorInFront, v => drone.isRotorInFront = v);

        GetControl("KalmanAccelNoise")?.Initialize("Accel Noise", 0, 1, 0, v => { var k = drone.CurrentTiltEstimator as KalmanFilterEstimator; if (k != null) k.accelerometerNoise = v; accelNoiseOverride = v; });
        GetControl("KalmanGyroNoise")?.Initialize("Gyro Noise", 0, 1, 0, v => { var k = drone.CurrentTiltEstimator as KalmanFilterEstimator; if (k != null) k.gyroscopeNoise = v; gyroNoiseOverride = v; });
        GetControl("KalmanQ_Angle")?.Initialize("Q Angle", 0, 0.1f, 0, v => { var k = drone.CurrentTiltEstimator as KalmanFilterEstimator; if (k != null) k.q_angle = v; qAngleOverride = v; });
        GetControl("KalmanQ_Rate")?.Initialize("Q Rate", 0, 0.1f, 0, v => { var k = drone.CurrentTiltEstimator as KalmanFilterEstimator; if (k != null) k.q_rate = v; qRateOverride = v; });
        GetControl("KalmanR_Angle")?.Initialize("R Angle", 0, 1, 0, v => { var k = drone.CurrentTiltEstimator as KalmanFilterEstimator; if (k != null) k.r_angle = v; rAngleOverride = v; });
        GetControl("KalmanR_Rate")?.Initialize("R Rate", 0, 1, 0, v => { var k = drone.CurrentTiltEstimator as KalmanFilterEstimator; if (k != null) k.r_rate = v; rRateOverride = v; });

        GetControl("MaxTiltAngle")?.Initialize("Max Tilt Angle", 0, 80, 0, v => { SetMaxTiltAngle(v); maxTiltAngleOverride = v; });
        GetControl("SupportGain")?.Initialize("Support Gain", 0, 5, 0, v => { var s = drone.CurrentTargetModifier as StabilizationSupportModifier; if (s != null) s.supportGain = v; supportGainOverride = v; });

        GetControl("MaxRotorForce")?.Initialize("Max Rotor Force", -100, 100, 0, v => { var c = drone.CurrentRotorForceCalculator as ConstrainedRotorForceCalculator; if (c != null) c.maxRotorForce = v; maxRotorForceOverride = v; });
        GetControl("MinRotorForce")?.Initialize("Min Rotor Force", -100, 100, 0, v => { var c = drone.CurrentRotorForceCalculator as ConstrainedRotorForceCalculator; if (c != null) c.minRotorForce = v; minRotorForceOverride = v; });

        GetControl("MaxRotorDelta")?.Initialize("Max Rotor Delta", 0, 5, 0, v => { if (drone.CurrentFluentRotor != null) drone.CurrentFluentRotor.maxRotorDelta = v; maxRotorDeltaOverride = v; });

        GetControl("RotorRadius")?.Initialize("Rotor Radius", 0.1f, 1, 0, v => { if (drone.CurrentGroundEffect != null) drone.CurrentGroundEffect.rotorRadius = v; rotorRadiusOverride = v; });
    }

    private void UpdateUIVisibility() {
        var kalman = drone.CurrentTiltEstimator as KalmanFilterEstimator;
        SetGroupInteractable("Kalman", kalman != null);
        if (kalman != null) {
            ApplyOverride(kalman, ref kalman.accelerometerNoise, accelNoiseOverride, "KalmanAccelNoise");
            ApplyOverride(kalman, ref kalman.gyroscopeNoise, gyroNoiseOverride, "KalmanGyroNoise");
            ApplyOverride(kalman, ref kalman.q_angle, qAngleOverride, "KalmanQ_Angle");
            ApplyOverride(kalman, ref kalman.q_rate, qRateOverride, "KalmanQ_Rate");
            ApplyOverride(kalman, ref kalman.r_angle, rAngleOverride, "KalmanR_Angle");
            ApplyOverride(kalman, ref kalman.r_rate, rRateOverride, "KalmanR_Rate");
        }

        var stabModifier = drone.CurrentTargetModifier as StabilizationSupportModifier;
        GetControl("SupportGain")?.SetInteractable(stabModifier != null);
        if (stabModifier != null) {
            ApplyOverride(stabModifier, ref stabModifier.supportGain, supportGainOverride, "SupportGain");
        }

        var defaultModifier = drone.CurrentTargetModifier as DefaultTargetModifier;
        bool hasTilt = stabModifier != null || defaultModifier != null;
        GetControl("MaxTiltAngle")?.SetInteractable(hasTilt);
        if (hasTilt) {
            if (maxTiltAngleOverride.HasValue) SetMaxTiltAngle(maxTiltAngleOverride.Value);
            GetControl("MaxTiltAngle")?.SetValue(GetMaxTiltAngle());
        }
        
        var constrCalc = drone.CurrentRotorForceCalculator as ConstrainedRotorForceCalculator;
        bool isConstr = constrCalc != null;
        GetControl("MaxRotorForce")?.SetInteractable(isConstr);
        GetControl("MinRotorForce")?.SetInteractable(isConstr);
        if (constrCalc != null) {
            ApplyOverride(constrCalc, ref constrCalc.maxRotorForce, maxRotorForceOverride, "MaxRotorForce");
            ApplyOverride(constrCalc, ref constrCalc.minRotorForce, minRotorForceOverride, "MinRotorForce");
        }

        GetControl("MaxRotorDelta")?.SetInteractable(drone.CurrentFluentRotor != null);
        if (drone.CurrentFluentRotor != null) ApplyOverride(drone.CurrentFluentRotor, ref drone.CurrentFluentRotor.maxRotorDelta, maxRotorDeltaOverride, "MaxRotorDelta");

        GetControl("RotorRadius")?.SetInteractable(drone.CurrentGroundEffect != null);
        if (drone.CurrentGroundEffect != null) ApplyOverride(drone.CurrentGroundEffect, ref drone.CurrentGroundEffect.rotorRadius, rotorRadiusOverride, "RotorRadius");
    }

    #region Helper Methods
    private void ApplyOverride<T>(T component, ref float componentField, float? overrideValue, string controlName)
    {
        float value = overrideValue.HasValue ? overrideValue.Value : componentField;
        componentField = value;
        GetControl(controlName)?.SetValue(value);
    }
    
    private void SetupMultiStrategyControl<T>(TMP_Dropdown dropdown, List<T> presets, T currentPreset, Action<T> setter) where T : ScriptableObject {
        if (dropdown == null) return;
        dropdown.ClearOptions();

        if (presets == null || presets.Count == 0)
        {
            dropdown.interactable = false;
            if (currentPreset != null)
            {
                setter(null);
            }
            return;
        }

        var options = presets.Select(p => p.name).ToList();
        dropdown.AddOptions(options);

        dropdown.onValueChanged.AddListener(index => {
            setter(presets[index]);
            UpdateUIVisibility();
        });

        int initialIndex = -1;
        if (currentPreset != null) {
            initialIndex = presets.FindIndex(p => p.name == currentPreset.name.Replace("(Clone)", ""));
        }
        
        if (initialIndex == -1) {
            initialIndex = 0;
            setter(presets[initialIndex]);
        }
        
        dropdown.value = initialIndex;
    }

    private void SetupToggleStrategyControl<T>(Toggle toggle, T preset, T currentPreset, Action<T> setter) where T : ScriptableObject {
        if (toggle == null) return;
        if (preset == null) {
            toggle.interactable = false;
            return;
        }
        toggle.isOn = currentPreset != null;
        toggle.onValueChanged.AddListener(isOn => {
            setter(isOn ? preset : null);
            UpdateUIVisibility();
        });
    }

    private ParameterControl GetControl(string name) {
        controlDict.TryGetValue(name, out var control);
        if (control == null) Debug.LogWarning($"ParameterControl named '{name}' not found in the dictionary.");
        return control;
    }

    private void SetGroupInteractable(string groupName, bool isInteractable) {
        foreach (var control in controlDict.Values.Where(c => c.name.StartsWith(groupName))) {
            control.SetInteractable(isInteractable);
        }
    }
    
    private void SetupLinkedToggle(Toggle toggle, bool initialValue, Action<bool> setter)
    {
        if (toggle == null) return;
        toggle.isOn = initialValue;
        toggle.onValueChanged.AddListener(x => setter(x));
    }
    #endregion

    #region Special Accessors
    private float GetMaxTiltAngle()
    {
        var s = drone.CurrentTargetModifier as StabilizationSupportModifier; if (s != null) return s.maxTiltAngle;
        var d = drone.CurrentTargetModifier as DefaultTargetModifier; if (d != null) return d.maxTiltAngle;
        return 0;
    }

    private void SetMaxTiltAngle(float value)
    {
        var s = drone.CurrentTargetModifier as StabilizationSupportModifier; if (s != null) s.maxTiltAngle = value;
        var d = drone.CurrentTargetModifier as DefaultTargetModifier; if (d != null) d.maxTiltAngle = value;
    }
    #endregion
}


