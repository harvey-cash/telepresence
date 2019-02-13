using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotEye : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        GetComponent<Camera>().cullingMask = LayerMask.GetMask("Elsewhere");
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
