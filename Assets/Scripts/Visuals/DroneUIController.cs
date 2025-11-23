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

    [Header("Core Parameters")]
    public ParameterControl tiltGain;
    public ParameterControl tiltDamping;
    public ParameterControl yawGain;
    public ParameterControl maxYawRate;
    public ParameterControl maxVerticalVelocity;
    public ParameterControl verticalGain;
    public ParameterControl thrustMultiplier;
    public ParameterControl dragMultiplier;
    public ParameterControl airResistance;
    public ParameterControl rotorDistance;
    public Toggle isRotorInFrontToggle;

    [Header("Kalman Filter Parameters")]
    public ParameterControl kalmanAccelNoise;
    public ParameterControl kalmanGyroNoise;
    public ParameterControl kalmanQAngle;
    public ParameterControl kalmanQRate;
    public ParameterControl kalmanRAngle;
    public ParameterControl kalmanRRate;
    
    [Header("Target Modifier Parameters")]
    public ParameterControl maxTiltAngle;
    public ParameterControl supportGain;
    
    [Header("Rotor Force Calculator Parameters")]
    public ParameterControl maxRotorForce;
    public ParameterControl minRotorForce;

    [Header("Fluent Rotor Smoother Parameters")]
    public ParameterControl maxRotorDelta;

    [Header("Ground Effect Parameters")]
    public ParameterControl rotorRadius;
    
    private readonly List<Parameter> allParameters = new List<Parameter>();

    void Awake() {
        drone = drone ?? FindFirstObjectByType<Drone>();
    }

    void Start() {
        if (drone == null) {
            Debug.LogError("Drone reference not set in DroneUIController.", this);
            return;
        }

        SetupParameters();
        
        foreach (var parameter in allParameters)
        {
            parameter.InitializeUI();
        }

        SetupStrategyControls();
        UpdateUIVisibility();
    }

    private void SetupStrategyControls() {
        SetupMultiStrategyControl(tiltEstimatorDropdown, availableTiltEstimators.Cast<ScriptableObject>().ToList(), drone.CurrentTiltEstimator, (p) => drone.SetTiltEstimator(p as TiltEstimator));
        SetupMultiStrategyControl(targetModifierDropdown, availableTargetModifiers.Cast<ScriptableObject>().ToList(), drone.CurrentTargetModifier, (p) => drone.SetTargetModifier(p as TargetModifier));
        SetupMultiStrategyControl(rotorCalculatorDropdown, availableRotorCalculators.Cast<ScriptableObject>().ToList(), drone.CurrentRotorForceCalculator, (p) => drone.SetRotorForceCalculator(p as RotorForceCalculator));

        SetupToggleStrategyControl(fluentRotorToggle, fluentRotorPreset, drone.CurrentFluentRotor, (p) => drone.SetFluentRotor(p));
        SetupToggleStrategyControl(groundEffectToggle, groundEffectPreset, drone.CurrentGroundEffect, (p) => drone.SetGroundEffect(p));
        SetupToggleStrategyControl(velocityInfluenceToggle, velocityInfluencePreset, drone.CurrentVelocityThrustInfluence, (p) => drone.SetVelocityThrustInfluence(p));
        
        SetupLinkedToggle(isRotorInFrontToggle, drone.isRotorInFront, v => drone.isRotorInFront = v);
    }
    
    private void SetupParameters()
    {
        void AddParam(Parameter param, ParameterControl control)
        {
            param.UIControl = control;
            allParameters.Add(param);
        }

        AddParam(new DroneFloatParameter("Tilt Gain", 0, 20, drone, d => d.tiltGain, (d, v) => d.tiltGain = v), tiltGain);
        AddParam(new DroneFloatParameter("Tilt Damping", 0, 10, drone, d => d.tiltDamping, (d, v) => d.tiltDamping = v), tiltDamping);
        AddParam(new DroneFloatParameter("Yaw Gain", 0, 10, drone, d => d.yawGain, (d, v) => d.yawGain = v), yawGain);
        AddParam(new DroneFloatParameter("Max Yaw Rate", 0, 10, drone, d => d.maxYawRate, (d, v) => d.maxYawRate = v), maxYawRate);
        AddParam(new DroneFloatParameter("Max Vert. Velocity", 0, 50, drone, d => d.maxVerticalVelocity, (d, v) => d.maxVerticalVelocity = v), maxVerticalVelocity);
        AddParam(new DroneFloatParameter("Vertical Gain", 0, 10, drone, d => d.verticalGain, (d, v) => d.verticalGain = v), verticalGain);
        AddParam(new DroneFloatParameter("Thrust Multiplier", 0, 5, drone, d => d.thrustMultiplier, (d, v) => d.thrustMultiplier = v), thrustMultiplier);
        AddParam(new DroneFloatParameter("Drag Multiplier", 0, 1, drone, d => d.dragMultiplier, (d, v) => d.dragMultiplier = v), dragMultiplier);
        AddParam(new DroneFloatParameter("Air Resistance", 0, 2, drone, d => d.airResistanceCoefficient, (d, v) => d.airResistanceCoefficient = v), airResistance);
        AddParam(new DroneFloatParameter("Rotor Distance", 0.1f, 5, drone, d => d.rotorDistance, (d, v) => d.rotorDistance = v), rotorDistance);

        AddParam(new StrategyParameter<KalmanFilterEstimator>("Accel Noise", 0, 1, drone, d => d.CurrentTiltEstimator as KalmanFilterEstimator, s => s.accelerometerNoise, (s, v) => s.accelerometerNoise = v), kalmanAccelNoise);
        AddParam(new StrategyParameter<KalmanFilterEstimator>("Gyro Noise", 0, 1, drone, d => d.CurrentTiltEstimator as KalmanFilterEstimator, s => s.gyroscopeNoise, (s, v) => s.gyroscopeNoise = v), kalmanGyroNoise);
        AddParam(new StrategyParameter<KalmanFilterEstimator>("Q Angle", 0, 0.1f, drone, d => d.CurrentTiltEstimator as KalmanFilterEstimator, s => s.q_angle, (s, v) => s.q_angle = v), kalmanQAngle);
        AddParam(new StrategyParameter<KalmanFilterEstimator>("Q Rate", 0, 0.1f, drone, d => d.CurrentTiltEstimator as KalmanFilterEstimator, s => s.q_rate, (s, v) => s.q_rate = v), kalmanQRate);
        AddParam(new StrategyParameter<KalmanFilterEstimator>("R Angle", 0, 1, drone, d => d.CurrentTiltEstimator as KalmanFilterEstimator, s => s.r_angle, (s, v) => s.r_angle = v), kalmanRAngle);
        AddParam(new StrategyParameter<KalmanFilterEstimator>("R Rate", 0, 1, drone, d => d.CurrentTiltEstimator as KalmanFilterEstimator, s => s.r_rate, (s, v) => s.r_rate = v), kalmanRRate);

        AddParam(new MaxTiltAngleParameter("Max Tilt Angle", 0, 80, drone), maxTiltAngle);
        AddParam(new StrategyParameter<StabilizationSupportModifier>("Support Gain", 0, 1, drone, d => d.CurrentTargetModifier as StabilizationSupportModifier, s => s.supportGain, (s, v) => s.supportGain = v), supportGain);

        AddParam(new StrategyParameter<ConstrainedRotorForceCalculator>("Max Rotor Force", -100, 100, drone, d => d.CurrentRotorForceCalculator as ConstrainedRotorForceCalculator, s => s.maxRotorForce, (s, v) => s.maxRotorForce = v), maxRotorForce);
        AddParam(new StrategyParameter<ConstrainedRotorForceCalculator>("Min Rotor Force", -100, 100, drone, d => d.CurrentRotorForceCalculator as ConstrainedRotorForceCalculator, s => s.minRotorForce, (s, v) => s.minRotorForce = v), minRotorForce);

        AddParam(new StrategyParameter<FluentRotorSmoother>("Max Rotor Delta", 0, 5, drone, d => d.CurrentFluentRotor, s => s.maxRotorDelta, (s, v) => s.maxRotorDelta = v), maxRotorDelta);

        AddParam(new StrategyParameter<GroundEffect>("Rotor Radius", 0.1f, 1, drone, d => d.CurrentGroundEffect, s => s.rotorRadius, (s, v) => s.rotorRadius = v), rotorRadius);
    }

    private void UpdateUIVisibility() {
        foreach (var parameter in allParameters)
        {
            parameter.Sync();
        }
    }

    #region Helper Methods
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

    private void SetupLinkedToggle(Toggle toggle, bool initialValue, Action<bool> setter)
    {
        if (toggle == null) return;
        toggle.isOn = initialValue;
        toggle.onValueChanged.AddListener(x => setter(x));
    }
    #endregion
}


