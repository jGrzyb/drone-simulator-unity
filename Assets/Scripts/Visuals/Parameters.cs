using System;
using UnityEngine;

public abstract class Parameter
{
    public string Label { get; }
    public float MinValue { get; }
    public float MaxValue { get; }
    protected Drone drone;

    public ParameterControl UIControl { get; set; }

    protected Parameter(string label, float minValue, float maxValue, Drone drone)
    {
        this.Label = label;
        this.MinValue = minValue;
        this.MaxValue = maxValue;
        this.drone = drone;
    }

    public abstract float GetValue();

    public abstract void SetValue(float value);

    public abstract bool IsAvailable();

    protected void UpdateUI()
    {
        if (UIControl == null) return;

        bool isAvailable = IsAvailable();
        UIControl.SetInteractable(isAvailable);
        if (isAvailable)
        {
            UIControl.SetValue(GetValue());
        }
    }

    public void InitializeUI()
    {
        UIControl?.Initialize(Label, MinValue, MaxValue, GetValue(), SetValue);
    }
    
    public virtual void Sync()
    {
        UpdateUI();
    }
}

public class DroneFloatParameter : Parameter
{
    private readonly Func<Drone, float> _getter;
    private readonly Action<Drone, float> _setter;

    public DroneFloatParameter(string label, float min, float max, Drone drone, Func<Drone, float> getter, Action<Drone, float> setter)
        : base(label, min, max, drone)
    {
        _getter = getter;
        _setter = setter;
    }

    public override float GetValue() => _getter(drone);
    public override void SetValue(float value) => _setter(drone, value);
    public override bool IsAvailable() => true;
}

public class StrategyParameter<TStrategy> : Parameter where TStrategy : class
{
    private readonly Func<Drone, TStrategy> _strategyGetter;
    private readonly Func<TStrategy, float> _paramGetter;
    private readonly Action<TStrategy, float> _paramSetter;
    private float? _overrideValue;
    private TStrategy _lastKnownStrategy;

    public StrategyParameter(string label, float min, float max, Drone drone,
                             Func<Drone, TStrategy> strategyGetter,
                             Func<TStrategy, float> paramGetter,
                             Action<TStrategy, float> paramSetter)
        : base(label, min, max, drone)
    {
        _strategyGetter = strategyGetter;
        _paramGetter = paramGetter;
        _paramSetter = paramSetter;
    }

    public override bool IsAvailable() => _strategyGetter(drone) != null;

    public override float GetValue()
    {
        var strategy = _strategyGetter(drone);
        if (strategy == null) return default;
        return _overrideValue ?? _paramGetter(strategy);
    }

    public override void SetValue(float value)
    {
        _overrideValue = value;
        var strategy = _strategyGetter(drone);
        if (strategy != null)
        {
            _paramSetter(strategy, value);
        }
    }

    public override void Sync()
    {
        var currentStrategy = _strategyGetter(drone);
        
        if (currentStrategy != null && !ReferenceEquals(currentStrategy, _lastKnownStrategy))
        {
            if (_overrideValue.HasValue)
            {
                _paramSetter(currentStrategy, _overrideValue.Value);
            }
        }
        _lastKnownStrategy = currentStrategy;

        base.Sync();
    }
}

public class MaxTiltAngleParameter : Parameter
{
    private float? _overrideValue;
    private object _lastKnownStrategy;

    public MaxTiltAngleParameter(string label, float min, float max, Drone drone)
        : base(label, min, max, drone) { }

    public override bool IsAvailable()
    {
        return drone.CurrentTargetModifier is StabilizationSupportModifier ||
               drone.CurrentTargetModifier is DefaultTargetModifier;
    }

    public override float GetValue()
    {
        if (_overrideValue.HasValue) return _overrideValue.Value;

        if (drone.CurrentTargetModifier is StabilizationSupportModifier s) return s.maxTiltAngle;
        if (drone.CurrentTargetModifier is DefaultTargetModifier d) return d.maxTiltAngle;
        return default;
    }

    public override void SetValue(float value)
    {
        _overrideValue = value;
        if (drone.CurrentTargetModifier is StabilizationSupportModifier s) s.maxTiltAngle = value;
        if (drone.CurrentTargetModifier is DefaultTargetModifier d) d.maxTiltAngle = value;
    }

    public override void Sync()
    {
        var currentStrategy = drone.CurrentTargetModifier as object;

        if (currentStrategy != null && !ReferenceEquals(currentStrategy, _lastKnownStrategy))
        {
            if (_overrideValue.HasValue)
            {
                SetValue(_overrideValue.Value);
            }
        }
        _lastKnownStrategy = currentStrategy;

        base.Sync();
    }
}
