using System;
using System.Collections.Generic;
using UnityEngine;

public class ForceTest : MonoBehaviour
{
    [SerializeField] private List<Force> forces = new();
    Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        foreach (var f in forces)
        {
            rb.AddForceAtPosition(transform.up * f.magnitude, transform.TransformPoint(f.position));        
            Debug.DrawLine(transform.TransformPoint(f.position), transform.TransformPoint(f.position) + transform.up * f.magnitude);
        }
    }

    [Serializable]
    public struct Force {
        public Vector3 position;
        public float magnitude;
    }
}
