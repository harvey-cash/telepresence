using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class TelepresenceRobot : MonoBehaviour
{
    // VR Headset Pose
    public abstract void ReceiveHeadPose(float timestamp, Pose headPose);

    public abstract void WheelVel(float left, float right);
}
