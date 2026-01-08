

using UnityEngine;

using Vector3 = UnityEngine.Vector3;


[CreateAssetMenu(fileName = "Kalman Tilt Estimator", menuName = "Drone/Tilt Estimators/Kalman Tilt Estimator")]
public class KalmanTiltEstimator : ITiltEstimator {
    [SerializeField] private float accelerometerNoise = 0.1f;
    [SerializeField] private float gyroscopeNoise = 0.1f;
    [SerializeField] private float qAngle = 0.001f;
    [SerializeField] private float qRate = 0.003f;
    [SerializeField] private float rAngle = 0.03f;
    [SerializeField] private float rRate = 0.03f;
    [SerializeField] private float gyroBiasDrift = 0.1f;
    [SerializeField] private float gyroInitialBias = 0.2f;


    private Kalman rollFilter;
    private Kalman pitchFilter;

    public float rollBias { get; private set; }
    public float pitchBias { get; private set; }

    public float noisyRollRate { get; private set; }
    public float noisyPitchRate { get; private set; }
    public float noisyAccRoll { get; private set; }
    public float noisyAccPitch { get; private set; }


    public override void Initialize(Drone drone)
    {
        base.Initialize(drone);
        rollFilter = new Kalman(qAngle, qRate, rAngle, rRate);
        pitchFilter = new Kalman(qAngle, qRate, rAngle, rRate);
        rollBias = gyroInitialBias;

        UIManager.I.accelerometerNoiseSlider.onValueChanged.AddListener((value) => accelerometerNoise = value);
        UIManager.I.gyroscopeNoiseSlider.onValueChanged.AddListener((value) => gyroscopeNoise = value);
        UIManager.I.gyroBiasDriftSlider.onValueChanged.AddListener((value) => gyroBiasDrift = value);
    }

    public override void UpdateFilter() {
        rollBias += Random.Range(-gyroBiasDrift, gyroBiasDrift) * Time.fixedDeltaTime;
        pitchBias += Random.Range(-gyroBiasDrift, gyroBiasDrift) * Time.fixedDeltaTime;


        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);

        noisyRollRate = -localAngularVelocity.z + rollBias + GetRandomNoise(gyroscopeNoise);
        noisyPitchRate = localAngularVelocity.x + pitchBias + GetRandomNoise(gyroscopeNoise);


        Vector3 localGravity = transform.InverseTransformDirection(Physics.gravity.normalized);

        float accRoll = Mathf.Atan2(localGravity.x, -localGravity.y);
        float accPitch = Mathf.Atan2(localGravity.z, -localGravity.y);

        noisyAccRoll = accRoll + GetRandomNoise(accelerometerNoise);
        noisyAccPitch = accPitch + GetRandomNoise(accelerometerNoise);


        rollFilter.Update(noisyAccRoll, noisyRollRate, Time.fixedDeltaTime);
        pitchFilter.Update(noisyAccPitch, noisyPitchRate, Time.fixedDeltaTime);
    }

    public override float GetRollAngle() {
        return (float)rollFilter.GetAngle() * Mathf.Rad2Deg;
    }

    public override float GetPitchAngle() {
        return (float)pitchFilter.GetAngle() * Mathf.Rad2Deg;
    }

    public override float GetRollRate() {
        return (float)rollFilter.GetRate(noisyRollRate) * Mathf.Rad2Deg;
    }

    public override float GetPitchRate() {
        return (float)pitchFilter.GetRate(noisyPitchRate) * Mathf.Rad2Deg;
    }

    private float GetRandomNoise(float range) {
        return Random.Range(-range, range);
    }
}
