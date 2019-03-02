using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class User : MonoBehaviour {

    public VirtualDisplay display;
    public Viewer viewer;
    public ImageStitcher stitcher;
    public Stabilisation stabilisation;

    private void Awake() {
        // Join the network
        Network.Join(this);
    }

    private void Start() {
        display.SetUser(this);

        // Create stitcher and stabiliser
        stitcher = new ImageStitcher(display);
        stabilisation = new Stabilisation(display);

        // Post head pose
        InvokeRepeating("PostHeadPose", 0, Config.POST_HEAD_POSE_MS / 1000f);
    }

    // Send head pose over the Network
    private void PostHeadPose() {
        Pose headPose = viewer.GetHeadPose();
        System.Action<float, Pose> target = Network.Robot.ReceiveHeadPose;

        StartCoroutine(Network.Post(target, Time.time / 1000f, headPose));
    }

    // Robot Imagery is passed to the ImageStitcher and to Stabilisation
    public void ReceiveImageryAndPose(float timestamp, byte[] left, byte[] right, Pose pose) {
        // Center display on user?
        if (Config.CENTER_DISPLAY_ON_HEAD) {
            pose = new Pose(viewer.transform.position, pose.rotation);
        }

        stitcher.StitchThenRender(timestamp, left, right, pose);
        stabilisation.Stabilise(timestamp, left, right);
    }
}
