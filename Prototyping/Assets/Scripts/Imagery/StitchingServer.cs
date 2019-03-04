using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using UnityEngine;

public class StitchingServer : MonoBehaviour
{
    private Thread thread;
    protected bool Running { get; private set; }
    protected bool Waiting { get; private set; }

    private float timestamp;
    private byte[] left, right;
    private Pose pose;

    private void Awake() {
        // Join the network
        Network.Join(this);
    }

    private void Start() {
        Running = true;

        thread = new Thread(
          () => {
              ForceDotNet.Force(); // this line is needed to prevent unity freeze after one use, not sure why yet
              using (RequestSocket client = new RequestSocket()) {
                  client.Connect("tcp://localhost:5555");

                  MemoryStream stream = new MemoryStream();
                  BinaryWriter bw = new BinaryWriter(stream);
                  BinaryReader br = new BinaryReader(stream);

                  while (Running) {
                      while (Waiting) {} // Block until we are ready to post image

                      bw.Write(timestamp);
                      bw.Flush();
                      byte[] floatBytes = stream.ToArray();

                      client.SendFrame(floatBytes);

                      byte[] message = null;
                      bool gotMessage = false;

                      while (Running) {
                          gotMessage = client.TryReceiveFrameBytes(out message); // this returns true if it's successful
                          if (gotMessage) break;
                      }

                      br.Dispose();
                      br.Read(message, 0, message.Length);

                      ReceiveStitchedBack((float)br.ReadDouble());
                  }
              }

              NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet

          });
        thread.Start();
    }

    // Robot Imagery is sent to the Python Server
    public void ReceiveImageryAndPose(float timestamp, byte[] left, byte[] right, Pose pose) {
        this.timestamp = timestamp;
        this.left = left;
        this.right = right;
        this.pose = pose;
        Waiting = false;
    }

    private void ReceiveStitchedBack(float message) { //float timestamp, byte[] stitched, Pose pose) {
        Waiting = true;
        Debug.Log("Received " + message.ToString());
    }
}
