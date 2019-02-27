using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class TelepresenceRobot : MonoBehaviour
{
    // Ensure we join the network
    private void Awake() {
        Network.Join(this);
    }

    // VR Headset Pose
    public abstract void ReceiveHeadPose(float timestamp, Pose headPose);
}
