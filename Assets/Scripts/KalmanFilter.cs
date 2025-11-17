

using UnityEngine;
using Accord.Math;

using Vector3 = UnityEngine.Vector3;

public class Kalman {

    private double[] x;

    private double[,] P;

    private double[,] F;

    private double[,] Q;

    private double[,] H;

    private double[,] R;

    private double[,] I;

    public Kalman(double q_angle, double q_rate, double r_angle, double r_rate) {
        x = new double[] { 0, 0 };

        P = new double[,] {
            { 1000, 0 },
            { 0, 1000 }
        };

        F = new double[2, 2];

        Q = new double[,] {
            { q_angle, 0 },
            { 0, q_rate }
        };

        H = Matrix.Identity(2);

        R = new double[,] {
            { r_angle, 0 },
            { 0, r_rate }
        };

        I = Matrix.Identity(2);
    }

    public double[] Update(double angle_measurement, double rate_measurement, float dt) {

        F[0, 0] = 1; F[0, 1] = dt;
        F[1, 0] = 0; F[1, 1] = 1;

        x = Matrix.Dot(F, x);

        double[,] p_predicted = Elementwise.Add(Matrix.Dot(Matrix.Dot(F, P), F.Transpose()), Q);



        double[] z = { angle_measurement, rate_measurement };

        double[] y = Elementwise.Subtract(z, Matrix.Dot(H, x));

        double[,] S = Elementwise.Add(Matrix.Dot(Matrix.Dot(H, p_predicted), H.Transpose()), R);

        double[,] K = Matrix.Dot(Matrix.Dot(p_predicted, H.Transpose()), S.Inverse());

        x = Elementwise.Add(x, Matrix.Dot(K, y));

        P = Matrix.Dot(Elementwise.Subtract(I, Matrix.Dot(K, H)), p_predicted);

        return x;
    }
}


[RequireComponent(typeof(Rigidbody))]
public class KalmanFilter : MonoBehaviour {
    [Header("Sensor Simulation")]
    [Tooltip("Noise added to the accelerometer readings.")]
    [SerializeField] private float accelerometerNoise = 0.1f;
    [Tooltip("Noise added to the gyroscope readings.")]
    [SerializeField] private float gyroscopeNoise = 0.1f;

    [Header("Kalman Filter Parameters")]
    [Tooltip("Process noise for the angle. Represents uncertainty in the physics model.")]
    [SerializeField] private float q_angle = 0.001f;
    [Tooltip("Process noise for the angular rate. Represents uncertainty in the physics model.")]
    [SerializeField] private float q_rate = 0.003f;
    [Tooltip("Measurement noise for the angle. Represents how much you trust the accelerometer.")]
    [SerializeField] private float r_angle = 0.03f;
    [Tooltip("Measurement noise for the rate. Represents how much you trust the gyroscope.")]
    [SerializeField] private float r_rate = 0.03f;


    public float RollAngle { get; private set; }
    public float RollRate { get; private set; }
    public float PitchAngle { get; private set; }
    public float PitchRate { get; private set; }

    private Rigidbody rb;

    private Kalman rollFilter;
    private Kalman pitchFilter;

    void Start() {
        rb = GetComponent<Rigidbody>();

        rollFilter = new Kalman(q_angle, q_rate, r_angle, r_rate);
        pitchFilter = new Kalman(q_angle, q_rate, r_angle, r_rate);
    }

    void FixedUpdate() {

        Vector3 localAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);

        float noisyRollRate = localAngularVelocity.z + GetRandomNoise(gyroscopeNoise);
        float noisyPitchRate = localAngularVelocity.x + GetRandomNoise(gyroscopeNoise);

        Vector3 localGravity = transform.InverseTransformDirection(Physics.gravity.normalized);

        float accRoll = Mathf.Atan2(localGravity.x, -localGravity.y);
        float accPitch = Mathf.Atan2(localGravity.z, -localGravity.y);

        float noisyAccRoll = accRoll + GetRandomNoise(accelerometerNoise);
        float noisyAccPitch = accPitch + GetRandomNoise(accelerometerNoise);



        double[] rollState = rollFilter.Update(noisyAccRoll, noisyRollRate, Time.fixedDeltaTime);
        RollAngle = (float)rollState[0] * Mathf.Rad2Deg;
        RollRate = (float)rollState[1] * Mathf.Rad2Deg;

        double[] pitchState = pitchFilter.Update(noisyAccPitch, noisyPitchRate, Time.fixedDeltaTime);
        PitchAngle = (float)pitchState[0] * Mathf.Rad2Deg;
        PitchRate = (float)pitchState[1] * Mathf.Rad2Deg;


        Debug.Log($"Roll -> Acc: {noisyAccRoll * Mathf.Rad2Deg:F2}, Gyro: {noisyRollRate * Mathf.Rad2Deg:F2}, Filtered: {RollAngle:F2}");
        Debug.Log($"Pitch -> Acc: {noisyAccPitch * Mathf.Rad2Deg:F2}, Gyro: {noisyPitchRate * Mathf.Rad2Deg:F2}, Filtered: {PitchAngle:F2}");
    }

    private float GetRandomNoise(float range) {
        return Random.Range(-range, range);
    }
}
