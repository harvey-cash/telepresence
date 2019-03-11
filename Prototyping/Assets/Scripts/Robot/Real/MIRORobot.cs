using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MIRORobot : TelepresenceRobot
{
    private SocketPublisher miroComms;

    // Start is called before the first frame update
    void Start()
    {
        miroComms = new SocketPublisher();
        miroComms.Start();
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
        float[] targetAngles = NeckKinematics.FindRadAngles(headPose);

        string jsonAngles = "{\"lift\": " + targetAngles[0] + ", \"yaw\": " + targetAngles[1] + ", \"pitch\": " + targetAngles[2] + "}";
        miroComms.targetAngles = jsonAngles;
    }
}
