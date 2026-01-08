using System.IO;
using System.Text;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Profiling;

public class PerformanceMonitor : MonoBehaviour {
    bool isMeasuring = false;

    private ProfilerRecorder mainThreadRecorder;
    private ProfilerRecorder gcAllocRecorder;
    private ProfilerRecorder droneScriptRecorder;

    private int fixedFrameCounter = 0;

    private StringBuilder sb = new();
    private StringBuilder fixedSb = new();


    public void HandleMesurement() {
        if (!isMeasuring) {
            isMeasuring = true;
            StartMeasurement();
        } else {
            isMeasuring = false;
            EndMeasurement(FindFirstObjectByType<Drone>());
        }
    }


    public void StartMeasurement() {
        Debug.Log("Starting performance measurement...");

        sb.Clear();
        sb.AppendLine("Time,DeltaTime,MainThreadTime,GCAlloc,TotalMemory,DroneFixedUpdateTime");
        fixedSb.Clear();
        fixedSb.AppendLine("FixedFrame,Time,LeftJoystickX,LeftJoystickY,RightJoystickX,RightJoystickY," +
            "TrueRollAngle,TruePitchAngle,TrueRollRate,TruePitchRate," +
            "MeasuredRollAngle,MeasuredPitchAngle,MeasuredRollRate,MeasuredPitchRate," +
            "NoisyAccRoll,NoisyAccPitch,NoisyRollRate,NoisyPitchRate,RollBias,PitchBias," +
            "PositionX,PositionY,PositionZ," +
            "LocalLinearVelocityX,LocalLinearVelocityY,LocalLinearVelocityZ," +
            "LocalAngularVelocityX,LocalAngularVelocityY,LocalAngularVelocityZ," +
            "RotorForce0,RotorForce1,RotorForce2,RotorForce3");

        fixedFrameCounter = 0;

        mainThreadRecorder = ProfilerRecorder.StartNew(
            ProfilerCategory.Internal,
            "Main Thread");

        gcAllocRecorder = ProfilerRecorder.StartNew(
            ProfilerCategory.Memory,
            "GC Allocated In Frame");

        droneScriptRecorder = ProfilerRecorder.StartNew(
            ProfilerCategory.Scripts,
            "DroneFixedUpdate");
    }

    void Update() {
        if (!isMeasuring) return;
        long ramBytes = Profiler.GetTotalAllocatedMemoryLong();
        sb.AppendLine($"{Time.time},{Time.unscaledDeltaTime},{mainThreadRecorder.LastValue},{gcAllocRecorder.LastValue},{ramBytes},{droneScriptRecorder.LastValue}");
    }

    public void FixedMesurement(Drone drone) {
        if (!isMeasuring) return;
        
        fixedFrameCounter++;
        
        Rigidbody rb = drone.GetComponent<Rigidbody>();
        Transform droneTransform = drone.transform;

        Vector2 leftJoystick = InputManager.I.leftJoystick;
        Vector2 rightJoystick = InputManager.I.rightJoystick;

        Vector3 localGravity = droneTransform.InverseTransformDirection(Physics.gravity.normalized);
        float trueRollAngle = Mathf.Atan2(localGravity.x, -localGravity.y) * Mathf.Rad2Deg;
        float truePitchAngle = Mathf.Atan2(localGravity.z, -localGravity.y) * Mathf.Rad2Deg;

        Vector3 localAngularVelocity = droneTransform.InverseTransformDirection(rb.angularVelocity);
        float trueRollRate = -localAngularVelocity.z * Mathf.Rad2Deg;
        float truePitchRate = localAngularVelocity.x * Mathf.Rad2Deg;

        float measuredRollAngle = drone.currentRoll * Mathf.Rad2Deg;
        float measuredPitchAngle = drone.currentPitch * Mathf.Rad2Deg;
        float measuredRollRate = drone.rollRate * Mathf.Rad2Deg;
        float measuredPitchRate = drone.pitchRate * Mathf.Rad2Deg;

        Vector3 droneLinearVelocity = rb.linearVelocity;
        Vector3 droneLocalLinearVelocity = droneTransform.InverseTransformDirection(droneLinearVelocity);
        Vector3 droneAngularVelocity = rb.angularVelocity;
        Vector3 droneLocalAngularVelocity = droneTransform.InverseTransformDirection(droneAngularVelocity);

        Vector3 dronePosition = droneTransform.position;

        float noisyAccRoll = drone.kalmanTiltEstimatorComponent.noisyAccRoll;
        float noisyAccPitch = drone.kalmanTiltEstimatorComponent.noisyAccPitch;
        float noisyRollRate = drone.kalmanTiltEstimatorComponent.noisyRollRate;
        float noisyPitchRate = drone.kalmanTiltEstimatorComponent.noisyPitchRate;
        float rollBias = drone.kalmanTiltEstimatorComponent.rollBias;
        float pitchBias = drone.kalmanTiltEstimatorComponent.pitchBias;

        float[] rotorForces = drone.rotorForcesArray;

        fixedSb.AppendLine($"{fixedFrameCounter},{Time.time},{leftJoystick.x},{leftJoystick.y},{rightJoystick.x},{rightJoystick.y}," +
            $"{trueRollAngle},{truePitchAngle},{trueRollRate},{truePitchRate}," +
            $"{measuredRollAngle},{measuredPitchAngle},{measuredRollRate},{measuredPitchRate}," +
            $"{noisyAccRoll},{noisyAccPitch},{noisyRollRate},{noisyPitchRate},{rollBias},{pitchBias}," +
            $"{dronePosition.x},{dronePosition.y},{dronePosition.z}," +
            $"{droneLocalLinearVelocity.x},{droneLocalLinearVelocity.y},{droneLocalLinearVelocity.z}," +
            $"{droneLocalAngularVelocity.x},{droneLocalAngularVelocity.y},{droneLocalAngularVelocity.z}," +
            $"{rotorForces[0]},{rotorForces[1]},{rotorForces[2]},{rotorForces[3]}");
    }

    public void EndMeasurement(Drone drone) {
        int droneCount = FindObjectsByType<Drone>(FindObjectsSortMode.None).Length;
        string timestamp = System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        string logsFolder = Path.Combine(Application.dataPath, "logs");
        if (!Directory.Exists(logsFolder)) {
            Directory.CreateDirectory(logsFolder);
        }
        string filePath = Path.Combine(logsFolder, $"performance_log_{droneCount}d_{timestamp}.csv");
        File.WriteAllText(filePath, sb.ToString());
        mainThreadRecorder.Dispose();
        gcAllocRecorder.Dispose();
        droneScriptRecorder.Dispose();
        Debug.Log($"Performance log saved to: {filePath}");

        string fixedFilePath = Path.Combine(logsFolder, $"fixed_performance_log_{droneCount}d_{timestamp}.csv");
        File.WriteAllText(fixedFilePath, fixedSb.ToString());
        Debug.Log($"Fixed performance log saved to: {fixedFilePath}");
    }
}
