using UnityEngine;

public class Camera : MonoBehaviour
{
    [SerializeField] GameObject obj;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.LookAt(obj.transform.position);
    }
}
