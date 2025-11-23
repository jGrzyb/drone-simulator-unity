using UnityEngine;
using Accord.Math;
using Accord.Math.Optimization;
using System.Linq;
using System.Collections.Generic;

[CreateAssetMenu(fileName = "ConstrainedRotorForceCalculator", menuName = "Drone/Modifiers/Rotor Force Calculator/Constrained")]
public class ConstrainedRotorForceCalculator : RotorForceCalculator
{
    [Tooltip("The minimum force each rotor can produce. Can be negative")]
    public float minRotorForce = 0.0f;
    [Tooltip("The maximum force each rotor can produce.")]
    public float maxRotorForce = 10.0f;

    public override float[] Calculate(Drone drone, float upwardForce, float rollControl, float pitchControl, float yawControl)
    {
        upwardForce = Mathf.Clamp(upwardForce, 4 * minRotorForce, 4 * maxRotorForce);
        double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };
        double[,] hMatrix = Matrix.Dot(drone.ControlToRotorMatrix.Transpose(), drone.ControlToRotorMatrix).Multiply(2);
        double[] fVector = Matrix.Dot(drone.ControlToRotorMatrix.Transpose(), controlInputArray).Multiply(-2);

        QuadraticObjectiveFunction qof = new QuadraticObjectiveFunction(hMatrix, fVector);

        var constraints = new List<LinearConstraint>();
        for (int i = 0; i < 4; i++)
        {
            var coeff = new double[4];
            coeff[i] = 1.0;
            constraints.Add(new LinearConstraint(4)
            {
                CombinedAs = coeff,
                ShouldBe = ConstraintType.GreaterThanOrEqualTo,
                Value = minRotorForce
            });
            constraints.Add(new LinearConstraint(4)
            {
                CombinedAs = coeff,
                ShouldBe = ConstraintType.LesserThanOrEqualTo,
                Value = maxRotorForce
            });
        }

        var solver = new GoldfarbIdnani(qof, constraints.ToArray());
        solver.Minimize();

        return solver.Solution.Select(x => (float)x).ToArray();
    }
}
