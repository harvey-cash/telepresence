using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MIRORobot : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        SocketPublisher socketPublisher = new SocketPublisher();
        socketPublisher.Start();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
