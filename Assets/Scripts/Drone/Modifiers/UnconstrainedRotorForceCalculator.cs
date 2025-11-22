using UnityEngine;
using Accord.Math;
using System.Linq;

[CreateAssetMenu(fileName = "UnconstrainedRotorForceCalculator", menuName = "Drone/Modifiers/Rotor Force Calculator/Unconstrained")]
public class UnconstrainedRotorForceCalculator : RotorForceCalculator
{
    public override float[] Calculate(Drone drone, float upwardForce, float rollControl, float pitchControl, float yawControl)
    {
        double[] controlInputArray = new double[] { upwardForce, rollControl, pitchControl, yawControl };
        return Matrix.Solve(drone.ControlToRotorMatrix, controlInputArray).Select(x => (float)x).ToArray();
    }
}
