using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StitchingServer : MonoBehaviour
{
    private StitchRequester stitchRequester;

    private void Awake() {
        // Join the network
        Network.Join(this);
    }

    private void Start() {
        stitchRequester = new StitchRequester { StitchingServer = this };

        // Runs on a separate thread, and waits for new data for stitching
        stitchRequester.Start();
    }

    // Robot Imagery is sent to the Python Server
    public void ReceiveImageryAndPose(float timestamp, byte[] left, byte[] right, Pose pose) {
        // Set new parameters
        stitchRequester.Timestamp = timestamp;
        stitchRequester.Left = left;
        stitchRequester.Right = right;
        stitchRequester.Pose = pose;

        // Flag as ready
        stitchRequester.startStitch = true;
    }
}
