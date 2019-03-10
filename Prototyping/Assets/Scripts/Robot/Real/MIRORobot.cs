using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MIRORobot : MonoBehaviour
{
    private SocketPublisher miroComms;

    // Start is called before the first frame update
    void Start()
    {
        miroComms = new SocketPublisher();
        miroComms.Start();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDestroy() {
        miroComms.Stop();
    }
}
