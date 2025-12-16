using System.Collections.Generic;
using System.Linq;
using Accord.Math;
using Accord.Math.Optimization;
using UnityEngine;

[CreateAssetMenu(fileName = "Constrained Control Allocator", menuName = "Drone/Control Allocators/Constrained Control Allocator")]
public class ConstrainedControlAllocator : IControlAllocator {
    [SerializeField] public float minRotorForce = 0.0f;
    [SerializeField] private float maxRotorForce = 10.0f;
    private LinearConstraintCollection linearConstraints;

    public override void Initialize() {
        base.Initialize();
        updateConstraints();
        UIManager.I.minRotorValueSlider.onValueChanged.AddListener(value => {minRotorForce = value; updateConstraints();});
        UIManager.I.maxRotorValueSlider.onValueChanged.AddListener(value => {maxRotorForce = value; updateConstraints();});
        
    }

    public override float[] Allocate(float upwardForce, float rollControl, float pitchControl, float yawControl, double[,] controlToRotorMatrix)
    {
        upwardForce = Mathf.Clamp(upwardForce, 4 * minRotorForce, 4 * maxRotorForce);
        double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };

        double[,] H = Matrix.Dot(controlToRotorMatrix.Transpose(), controlToRotorMatrix).Multiply(2);
        double[] g = Matrix.Dot(controlToRotorMatrix.Transpose(), controlInputArray).Multiply(-2);
        QuadraticObjectiveFunction qof = new QuadraticObjectiveFunction(H, g);
        
        var solver = new GoldfarbIdnani(qof, linearConstraints);
        solver.Minimize();
        return solver.Solution.Select(x => (float)x).ToArray();
    }

    private void updateConstraints() {
        var cons = new List<LinearConstraint>();
        for (int i = 0; i < 4; i++) {
            var coeff = new double[4]; coeff[i] = 1.0;
            cons.Add(new LinearConstraint(4) {
                CombinedAs = coeff,
                ShouldBe = ConstraintType.GreaterThanOrEqualTo,
                Value = minRotorForce
            });
            cons.Add(new LinearConstraint(4) {
                CombinedAs = coeff,
                ShouldBe = ConstraintType.LesserThanOrEqualTo,
                Value = maxRotorForce
            });
        }
        linearConstraints = new LinearConstraintCollection(cons);
    }
}