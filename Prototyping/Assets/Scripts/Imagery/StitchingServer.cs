using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using UnityEngine;

public class StitchingServer : MonoBehaviour {

    User user;
    StitchRequester stitchRequester;

    private void Awake() {
        Network.Join(this); // Join the network
    }

    private void Start() {
        user = Network.User;

        stitchRequester = new StitchRequester();
        stitchRequester.Start();
    }

    // Robot posts imagery to the server
    public void ReceiveImageryAndPose(float timestamp, byte[] left, byte[] right, Pose pose) {
        // RECEIVE: StitchingServer takes from receiveStitch and fills sendStitch --> SEND
        if (stitchRequester.stitchStatus == StitchRequester.StitchStatus.RECEIVE) {
            // Previous stitched out (temporary)
            ReceiveStitch receiveStitch = stitchRequester.receiveStitch;
            if (receiveStitch != null) {
                PostStitchedAndPose(receiveStitch.timestamp, receiveStitch.imagery, receiveStitch.pose);
            }

            // Current to-be-stitched in
            SendStitch sendStitch = new SendStitch(timestamp, pose, left, right);
            stitchRequester.sendStitch = sendStitch;

            // Update Finite State Machine
            stitchRequester.stitchStatus = StitchRequester.StitchStatus.SEND;
        }
        
    }

    // Get stitched imagery back from the server, send on to the user
    // No need to simulate network delay, as already accounted for by the robot posting
    // to the stitching server
    private void PostStitchedAndPose(float timestamp, byte[] stitched, Pose pose) {
        user.ReceiveImageryAndPose(timestamp, stitched, pose);
    }

    private void OnDestroy() {
        stitchRequester.Stop();
    }
}
