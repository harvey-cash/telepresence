using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class User : MonoBehaviour {

    public VirtualDisplay display;
    public Viewer viewer;

    private void Awake() {
        // Join the network
        Network.Join(this);
    }

    private void Start() {
        viewer = Network.Viewer;
        display = Network.Display;
        display.SetUser(this);

        // Post head pose
        InvokeRepeating("PostHeadPose", 0, Config.POST_HEAD_POSE_MS / 1000f);
    }

    private void Update() {
        float left = Input.GetAxis("Vertical");
        float right = Input.GetAxis("RightStick");
        System.Action<float, float> target = Network.Robot.WheelVel;

        StartCoroutine(Network.Post(target, left, right));
    }

    // Send head pose over the Network
    private void PostHeadPose() {
        Pose headPose = viewer.GetHeadPose();
        System.Action<float, Pose> target = Network.Robot.ReceiveHeadPose;

        StartCoroutine(Network.Post(target, Time.time / 1000f, headPose));
    }

    // Stitched Imagery is passed to the display for rendering
    public void ReceiveImageryAndPose(float timestamp, byte[] stitched, Pose pose) {
        // Center display on user?
        if (Config.CENTER_DISPLAY_ON_HEAD) {
            pose = new Pose(viewer.transform.position, pose.rotation);
        }

        display.Render(timestamp, stitched, pose);
    }
}
