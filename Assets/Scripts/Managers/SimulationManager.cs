using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SimulationManager : MonoBehaviour
{
    [SerializeField] private Drone dronePrefab;

    readonly private Dictionary<SimulationEnv, string> envSceneNames = new()
    {
        { SimulationEnv.Plane, "Plane" },
        { SimulationEnv.City, "Cube" },
        { SimulationEnv.Mountains, "Sphere" }
    };

    private SimulationEnv currentEnv = SimulationEnv.Plane;

    async void Start()
    {
        await ResetSimulation();
    }


    public async Task ResetSimulation()
    {
        Debug.Log("Resetting simulation...");
        int droneCountX = 1;
        int droneCountY = 1;
        int droneCountZ = 1;
        SimulationEnv env = SimulationEnv.Plane;
        FindFirstObjectByType<UIManager>()?.GetSimulationParameters(
            out droneCountX,
            out droneCountY,
            out droneCountZ,
            out env);
        if (SceneManager.GetSceneByName(envSceneNames[currentEnv]).isLoaded)
        {
            await SceneManager.UnloadSceneAsync(envSceneNames[currentEnv]);
        }
        currentEnv = env;
        string sceneName = envSceneNames[currentEnv];
        await SceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);
        SceneManager.SetActiveScene(SceneManager.GetSceneByName(sceneName));

        for (int x = 0; x < droneCountX; x++) {
            for (int y = 0; y < droneCountY; y++) {
                for (int z = 0; z < droneCountZ; z++) {
                    Vector3 position = new Vector3(x * 2.0f, 1.0f + y * 2.0f, z * 2.0f);
                    Drone drone = Instantiate(dronePrefab, position, Quaternion.identity);
                    if (y == droneCountY - 1 && x == droneCountX / 2 && z == droneCountZ / 2) {
                        FindFirstObjectByType<CameraManager>()?.SetTarget(drone);
                    }
                }
            }
        }
        CameraManager.I.ResetCameraPosition();
        UIManager.I.RefreshAllParameters();
    }

    public enum SimulationEnv
    {
        Plane,
        City,
        Mountains
    }
}
