using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* CODE ADAPTED FROM
 * https://github.com/off99555/Unity3D-Python-Communication
 */

public class StitchRequester : RunAbleThread {

    public StitchingServer StitchingServer { set; private get; }

    // To stitch through server
    public float Timestamp { set; private get; }
    public byte[] Left { set; private get; }
    public byte[] Right { set; private get; }
    public Pose Pose { set; private get; }

    // Main thread sets to true when above fields have been filled
    // Separate thread sets to false when finished
    public bool startStitch = false;

    protected override void Run() {
        ForceDotNet.Force(); // this line is needed to prevent unity freeze after one use, not sure why yet
        using (RequestSocket client = new RequestSocket()) {
            client.Connect("tcp://localhost:5555");

            client.SendFrame("hello");
            // ReceiveFrameString() blocks the thread until you receive the string, but TryReceiveFrameString()
            // do not block the thread, you can try commenting one and see what the other does, try to reason why
            // unity freezes when you use ReceiveFrameString() and play and stop the scene without running the server
            //                string message = client.ReceiveFrameString();
            //                Debug.Log("Received: " + message);
            string message = null;
            bool gotMessage = false;
            while (Running) {
                gotMessage = client.TryReceiveFrameString(out message); // this returns true if it's successful
                if (gotMessage) break;
            }

            if (gotMessage) Debug.Log("Received " + message);
        }

        NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet
    }


}
