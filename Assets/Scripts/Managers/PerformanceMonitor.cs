using System.IO;
using System.Text;
using Unity.Profiling;
using UnityEngine;
using UnityEngine.Profiling;

public class PerformanceMonitor : MonoBehaviour {
    [SerializeField]
    private int measurementDurationInFrames = 1000;

    private ProfilerRecorder mainThreadRecorder;
    private ProfilerRecorder gcAllocRecorder;

    private int framesLeft = -1;

    private StringBuilder sb = new();


    public void Measure() {
        Debug.Log("Starting performance measurement...");

        framesLeft = measurementDurationInFrames;
        sb.Clear();
        sb.AppendLine("Time,DeltaTime,MainThreadTime,GCAlloc,TotalMemory");
        mainThreadRecorder = ProfilerRecorder.StartNew(
            ProfilerCategory.Internal,
            "Main Thread");

        gcAllocRecorder = ProfilerRecorder.StartNew(
            ProfilerCategory.Memory,
            "GC Allocated In Frame");
    }

    void Update() {
        framesLeft--;
        if (framesLeft > 0) {
            long ramBytes = Profiler.GetTotalAllocatedMemoryLong();
            sb.AppendLine($"{Time.time},{Time.unscaledDeltaTime},{mainThreadRecorder.LastValue},{gcAllocRecorder.LastValue},{ramBytes}");
        } else if (framesLeft == 0) {
            int droneCount = FindObjectsByType<Drone>(FindObjectsSortMode.None).Length;
            string timestamp = System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
            string logsFolder = Path.Combine(Application.dataPath, "logs");
            if (!Directory.Exists(logsFolder)) {
                Directory.CreateDirectory(logsFolder);
            }
            string filePath = Path.Combine(logsFolder, $"performance_log_{droneCount}d_{measurementDurationInFrames}f_{timestamp}.csv");
            File.WriteAllText(filePath, sb.ToString());
            mainThreadRecorder.Dispose();
            gcAllocRecorder.Dispose();
            Debug.Log($"Performance log saved to: {filePath}");
        }
    }
}
