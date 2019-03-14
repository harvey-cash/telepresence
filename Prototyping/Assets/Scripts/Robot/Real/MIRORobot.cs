using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MIRORobot : TelepresenceRobot
{
    private SendHeadtracking miroComms;
    private RequestImagery miroImagery;
    private User user;

    // Start is called before the first frame update
    void Start()
    {
        user = Network.User;

        miroComms = new SendHeadtracking();
        miroImagery = new RequestImagery();
        miroComms.Start();
        miroImagery.Start();

        // Record camera imagery repeatedly
        // Imagery and Pose are automatically posted together
        InvokeRepeating("GetImagery", 1f, Config.ROBOT_FRAME_WAIT_MS / 1000f);
    }

    private void Awake() {
        if (Config.USE_MIRO_SERVER) {
            Network.Join(this);
            Debug.Log(this.name + " has joined the Network.");
        } else {
            Destroy(this);
        }
    }

    private void OnDestroy() {
        miroComms.Stop();
    }

    public override void ReceiveHeadPose(float timestamp, Pose headPose) {
        // USE INVERSE KINEMATICS TO GET ANGLES
        float[] targetRads = NeckKinematics.FindRadAngles(headPose);

        // Note that we take the negative target yaw, as MIRO is configured in the other direction
        string jsonAngles = "{\"lift\": " + targetRads[0] + ", \"yaw\": " + (-targetRads[1]) + ", \"pitch\": " + targetRads[2] + "}";
        miroComms.targetAngles = jsonAngles;
    }

    // Get stitched imagery back from the server, send on to the user
    // No need to simulate network delay, as already accounted for by the robot posting
    // to the stitching server
    private void GetImagery() {
        Pose pose = new Pose(Vector3.zero, miroImagery.poseRot);

        user.ReceiveImageryAndPose(Time.time, miroImagery.stitched, pose);
    }
}
