

using UnityEngine;
using Accord.Math;
using Accord.Statistics.Distributions.Univariate;

using Vector3 = UnityEngine.Vector3;

public class Kalman {
    public double[] x;
    private double[,] P;
    private double[,] Q;
    private double[,] H;
    private double[,] R;
    private double[,] I;

    public Kalman(double qAngle, double qBias, double rAngle, double rRate) {

        x = new double[] { 0, 0 };

        P = new double[,] {
            { 1000, 0 },
            { 0, 1000 }
        };

        Q = new double[,] {
            { qAngle, 0 },
            { 0, qBias }
        };

        H = new double[,] {
            { 1, 0 }
        };

        R = new double[,] {
            { rAngle, 0 },
            { 0, rRate }
        };

        I = Matrix.Identity(2);
    }

    public void Update(double angleAcc, double gyroRate, float dt) {
        double[] xPred = { x[0] + dt * (gyroRate - x[1]), x[1] };

        double[,] F = new double[,]
        {
            { 1, -dt },
            { 0, 1 }
        };

        double[,] pPred =
            Matrix.Dot(Matrix.Dot(F, P), F.Transpose()).Add(Q);

        double[,] S = Matrix.Dot(Matrix.Dot(H, pPred), H.Transpose()).Add(R);

        double[,] K = Matrix.Dot(Matrix.Dot(pPred, H.Transpose()), S.Inverse());

        double z = angleAcc;
        double[] y = { z - xPred[0] };

        x = xPred.Add(Matrix.Dot(K, y));

        P = Matrix.Dot(I.Subtract(Matrix.Dot(K, H)), pPred);
    }

    public double GetAngle() => x[0];
    public double GetBias() => x[1];


    public double GetRate(double gyroRate) {
        return gyroRate - x[1];
    }
}



[RequireComponent(typeof(Rigidbody))]
public class KalmanFilter : MonoBehaviour {
    [SerializeField] private float accelerometerNoise = 0.1f;
    [SerializeField] private float gyroscopeNoise = 0.1f;
    [SerializeField] private float qAngle = 0.001f;
    [SerializeField] private float qRate = 0.003f;
    [SerializeField] private float rAngle = 0.03f;
    [SerializeField] private float rRate = 0.03f;
    [SerializeField] private float gyroBiasDrift = 0.1f;
    [SerializeField] private float gyroInitialBias = 0.2f;

    private Rigidbody rb;

    private Kalman rollFilter;
    private Kalman pitchFilter;

    private float rollBias;
    private float pitchBias;

    private float noisyRollRate;
    private float noisyPitchRate;
    private float noisyAccRoll;
    private float noisyAccPitch;

    void Start() {
        rb = GetComponent<Rigidbody>();

        rollFilter = new Kalman(qAngle, qRate, rAngle, rRate);
        pitchFilter = new Kalman(qAngle, qRate, rAngle, rRate);
        rollBias = gyroInitialBias;
        pitchBias = gyroInitialBias;
    }

    void FixedUpdate() {

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

    public float GetRollAngle() {
        return (float)rollFilter.GetAngle() * Mathf.Rad2Deg;
    }

    public float GetPitchAngle() {
        return (float)pitchFilter.GetAngle() * Mathf.Rad2Deg;
    }

    public float GetRollRate() {
        return (float)rollFilter.GetRate(noisyRollRate) * Mathf.Rad2Deg;
    }

    public float GetPitchRate() {
        return (float)pitchFilter.GetRate(noisyPitchRate) * Mathf.Rad2Deg;
    }

    private float GetRandomNoise(float range) {
        return Mathf.Clamp((float)new NormalDistribution(0, range / 3).Generate(), -range, range);
    }
}
