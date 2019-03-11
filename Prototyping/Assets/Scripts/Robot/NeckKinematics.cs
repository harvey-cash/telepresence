﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class contains the logic for finding the angles
 * required to place the head at a specific pose
 */
public static class NeckKinematics {

    // Overload to get angles in radians
    public static float[] FindRadAngles(Pose targetPose) {
        float[] angles = FindAngles(targetPose);
        for (int i = 0; i < angles.Length; i++) {
            angles[i] = Mathf.Deg2Rad * angles[i];
        }
        return angles;
    }

    /* This method returns an array [lift, yaw, pitch] of target
     * angles for MIRO's head to match the user's as closely as
     * possible */
    public static float[] FindAngles(Pose targetPose) {
        // Test Movement
        /*
        float liftAngle = 0.5f * (Mathf.Sin(Time.time) + 1) * 50;
        float yawAngle = 50;
        float pitchAngle = 0.5f * (Mathf.Cos(Time.time) + 1) * 50;
        */

        // Simplistic tracking
        Vector3 headForward = targetPose.rotation * Vector3.forward;
        float yawAngle = Vector3.SignedAngle(Vector3.forward, headForward, Vector3.up) + 50; // account for range
        float pitchAngle = Vector3.SignedAngle(Vector3.up, headForward, Vector3.Cross(Vector3.up, headForward)) - 63; // account for offset

        float liftAngle = 0; // Keep as vertical as possible for simplistic tracking
        //float pitchAngle = 27; // Keep level

        return new float[] { liftAngle, yawAngle, pitchAngle };
    }

    /* This method calculates the robot head pose based on our
     * neck model and the current motor angles from the robot
     */
    // currentAngles = [lift, yaw, pitch]
    public static Pose GetHeadPose(float[] currentAngles) {
        // Rotation
        
        // Assume lift is vertical for now
        float pitchFromVertical = currentAngles[2] - 27; // At pitchAngle(27) head is 90 degrees from vertical
        float yawFromForward = currentAngles[1] - 50; // Account for range

        Quaternion rotation = Quaternion.Euler(0, yawFromForward, 0) * Quaternion.Euler(pitchFromVertical, 0, 0);
        Vector3 position = Vector3.zero; // ignore position for now

        return new Pose(position, rotation);
    }
}
