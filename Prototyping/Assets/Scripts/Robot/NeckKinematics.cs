using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class contains the logic for finding the angles
 * required to place the head at a specific pose
 */
public static class NeckKinematics {
    /* This method returns an array [lift, yaw, pitch] of target
     * angles for MIRO's head to match the user's as closely as
     * possible */
    public static float[] FindAngles(Pose targetPose) {
        // Simplistic Method

        float liftAngle = targetPose.rotation.eulerAngles.x;
        float yawAngle = targetPose.rotation.eulerAngles.y;
        float pitchAngle = 0;

        return new float[] { liftAngle, yawAngle, pitchAngle };
    }

    /* This method calculates the robot head pose based on our
     * neck model and the current motor angles from the robot
     */
    public static Pose GetHeadPose(float[] currentAngles) {
        return new Pose(Vector3.zero, Quaternion.identity); // TEMPORARY, NULL
    }
}
