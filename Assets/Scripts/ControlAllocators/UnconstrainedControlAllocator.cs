using System.Linq;
using Accord.Math;
using UnityEngine;

[CreateAssetMenu(fileName = "Unconstrained Control Allocator", menuName = "Drone/Control Allocators/Unconstrained Control Allocator")]
public class UnconstrainedControlAllocator : IControlAllocator {
    public override float[] Allocate(float upwardForce, float rollControl, float pitchControl, float yawControl, double[,] controlToRotorMatrix)
    {
        double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };
        return Matrix.Solve(controlToRotorMatrix, controlInputArray).Select(x => (float)x).ToArray();
    }
}