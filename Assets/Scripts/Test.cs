using System;
using UnityEngine;

public class Test : MonoBehaviour
{
    [SerializeField] float mult = 1f;
    Rigidbody rb;
    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        Time.timeScale = 0.2f;
    }

    void FixedUpdate()
    {
        float upDiff = Mathf.Acos(Vector3.Dot(Vector3.up, transform.up));
        float forwardTilt = Mathf.Acos(Vector3.Dot(Vector3.up, transform.forward));
        Vector3 direction = Vector3.Cross(Vector3.up, transform.forward);
        rb.AddTorque(upDiff * direction * mult);
        
        Debug.Log($"{upDiff * Mathf.Rad2Deg}        {forwardTilt * Mathf.Rad2Deg}        {direction}");

        Debug.DrawLine(transform.position, transform.position + transform.up, Color.green);
        Debug.DrawLine(transform.position, transform.position + Vector3.up, Color.darkGreen);
        Debug.DrawLine(transform.position, transform.position + transform.forward, Color.blue);
        Debug.DrawLine(transform.position, transform.position + Vector3.forward, Color.darkBlue);
    }
}
