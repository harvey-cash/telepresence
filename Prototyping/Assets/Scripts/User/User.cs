using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class User : MonoBehaviour {
    // Time in ms between posting head pose to robot
    public static float postHeadPoseDelay = 20f;

    public VirtualDisplay display;
    public Viewer viewer;
    public ImageStitcher stitcher;
    public Stabilisation stabilisation;

    private void Awake() {
        // Join the network
        Network.Join(this);
    }

    private void Start() {
        // Create stitcher and stabiliser
        stitcher = new ImageStitcher(display);
        stabilisation = new Stabilisation(display);

        // Post head pose
        InvokeRepeating("PostHeadPose", 0, postHeadPoseDelay / 1000f);
    }

    // Send head pose over the Network
    private void PostHeadPose() {
        Pose headPose = viewer.GetHeadPose();
        System.Action<float, Pose> target = Network.Robot.ReceiveHeadPose;

        StartCoroutine(Network.Post(target, Time.time / 1000f, headPose));
    }

    // Robot Head Pose is forwarded to the VirtualDisplay
    public void ReceiveRobotPose(float timestamp, Pose robotPose) {
        display.ReceiveRobotPose(timestamp, robotPose);
    }

    // Robot Imagery is passed to the ImageStitcher and to Stabilisation
    public void ReceiveCameraImagery(float timestamp, byte[] left, byte[] right) {
        stitcher.StitchThenRender(timestamp, left, right);
        stabilisation.Stabilise(timestamp, left, right);
    }
}
