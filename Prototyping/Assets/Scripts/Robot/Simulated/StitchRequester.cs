using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/* CODE ADAPTED FROM
 * https://github.com/off99555/Unity3D-Python-Communication
 */

public class StitchRequester : RunAbleThread {

    // Finite State Machine for server communication
    // stitchStatus is checked by both StitchingServer and StitchRequester
    // RECEIVE: StitchingServer takes from receiveStitch and fills sendStitch --> SEND
    // SEND: StitchRequester posts from sendStitch to server --> WAIT
    // WAIT: StitchRequester fills receiveStitch from server response --> RECEIVE
    public enum StitchStatus { SEND, WAIT, RECEIVE };
    public StitchStatus stitchStatus = StitchStatus.RECEIVE;
    public SendStitch sendStitch = null;
    public ReceiveStitch receiveStitch = null;

    protected override void Run() {

        ForceDotNet.Force(); // this line is needed to prevent unity freeze after one use, not sure why yet

        using (RequestSocket client = new RequestSocket()) {
            client.Connect("tcp://localhost:5555");

            while (Running) {

                // SEND: StitchRequester posts from sendStitch to server --> WAIT
                if (stitchStatus == StitchStatus.SEND) {
                    stitchStatus = StitchStatus.WAIT;
                    PostSendStitch(client);
                }
                
            }
        }

        NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet
    }

    private void PostSendStitch(RequestSocket client) {

        client.SendFrame(sendStitch.left); // Just left image for now
        float timestamp = sendStitch.timestamp;
        Pose pose = sendStitch.pose;

        // Wait for response
        byte[] response = null;
        bool gotResponse = false;
        while (Running) {
            gotResponse = client.TryReceiveFrameBytes(out response);
            if (gotResponse) { break; }
        }

        // WAIT: StitchRequester fills receiveStitch from server response --> RECEIVE
        receiveStitch = new ReceiveStitch(timestamp, pose, response);
        stitchStatus = StitchStatus.RECEIVE;
    }

    private void PostHello(RequestSocket client) {
        Debug.Log("Sending Hello");
        client.SendFrame("Hello");
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

}

public class SendStitch {

    public float timestamp;
    public Pose pose;
    public byte[] left, right;

    public SendStitch(float timestamp, Pose pose, byte[] left, byte[] right) {
        this.timestamp = timestamp;
        this.pose = pose;
        this.left = left;
        this.right = right;
    }
}

public class ReceiveStitch {

    public float timestamp;
    public Pose pose;
    public byte[] imagery;

    public ReceiveStitch(float timestamp, Pose pose, byte[] imagery) {
        this.timestamp = timestamp;
        this.pose = pose;
        this.imagery = imagery;
    }
}
