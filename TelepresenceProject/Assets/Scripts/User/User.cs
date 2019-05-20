using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class User : MonoBehaviour {
    public static User user;
    
    [SerializeField]
    public MODE mode;

    public VirtualDisplay display;
    public Viewer viewer;

    private Vector3 storeDisplayPos = Vector3.zero;
    public void SetStoreDisplayPos(Vector3 pos) { clampHeadPosition = false; storeDisplayPos = pos; }

    bool clampHeadPosition = true;

    private void Awake() {
        user = this;
        Debug.LogWarning("MODE IS: " + mode.ToString());

        // Join the network
        Network.Join(this);
    }

    private void Start() {
        viewer = Network.Viewer;
        display = Network.Display;
        display.SetUser(this);

        // Post head pose
        InvokeRepeating("PostHeadPose", 0, Config.POST_HEAD_POSE_MS / 1000f);

        // Pass callback for viewer to set display pos, after a few seconds wait
        if (!Config.CENTER_DISPLAY_ON_HEAD) {
            storeDisplayPos = viewer.GetHeadPose().position;
            viewer.SetDisplayPosition(SetStoreDisplayPos);
        }
    }

    private void Update() {

        if (Input.GetKeyUp(KeyCode.Alpha1)) {            
            mode = MODE.COUPLE_SLOW;
            display.ResetMode();
        }
        if (Input.GetKeyUp(KeyCode.Alpha2)) {            
            mode = MODE.DECOUPLE_SLOW;
            display.ResetMode();
        }

        /*
        float left = Input.GetAxis("Vertical");
        float right = Input.GetAxis("RightStick");
        System.Action<float, float> target = Network.Robot.WheelVel;

        StartCoroutine(Network.Post(target, left, right));
        */
    }

    // Send head pose over the Network
    private void PostHeadPose() {
        Pose headPose = viewer.GetHeadPose();
        System.Action<float, Pose> target = Network.Robot.ReceiveHeadPose;

        StartCoroutine(Network.Post(target, Time.time / 1000f, headPose));
    }

    // Stitched Imagery is passed to the display for rendering
    public void ReceiveImageryAndPose(float timestamp, RenderTexture renderTexture, Pose pose) {
        // Center display on user?
        if (!Config.CENTER_DISPLAY_ON_HEAD && !clampHeadPosition) {
            // TODO: Account for robot head position change here?
            pose = new Pose(storeDisplayPos, pose.rotation);
        }
        else {
            pose = new Pose(viewer.GetHeadPose().position, pose.rotation);
        }

        display.ReceiveImageryAndPose(timestamp, renderTexture, pose);
    }

    // Stitched Imagery is passed to the display for rendering
    public void ReceiveImageryAndPose(float timestamp, byte[] imagery, Pose pose) {
        // Center display on user?
        if (!Config.CENTER_DISPLAY_ON_HEAD && !clampHeadPosition) {
            pose = new Pose(storeDisplayPos, pose.rotation);
        }
        else {
            pose = new Pose(viewer.GetHeadPose().position, pose.rotation);
        }

        throw new Exception("No implementation for byte[] imagery anymore!");
    }
}
