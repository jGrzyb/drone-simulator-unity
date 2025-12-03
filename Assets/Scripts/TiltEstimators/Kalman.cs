
using Accord.Math;

public class Kalman
{
    public double[] x;
    private double[,] P;
    private double[,] Q;
    private double[,] H;
    private double[,] R;
    private double[,] I;

    public Kalman(double qAngle, double qBias, double rAngle, double rRate)
    {

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

    public void Update(double angleAcc, double gyroRate, float dt)
    {
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


    public double GetRate(double gyroRate)
    {
        return gyroRate - x[1];
    }
}