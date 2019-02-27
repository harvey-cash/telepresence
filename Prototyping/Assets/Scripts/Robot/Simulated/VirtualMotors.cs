using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class is responsible for moving the GameObjects of the
 * simulated MiRo, in accordance with the real MiRo's neck model
 */
public class VirtualMotors : MonoBehaviour {
    /* MIRO has three "kinematic" degrees-of-freedom (DOF), which are denoted LIFT, YAW and PITCH. 
     * These three DOFs connect together MIRO's four physical links (and frames of reference), 
     * denoted BODY, NECK, GIMBAL and HEAD.
     */

    public Transform body, neck, gimbal, head;

    // Limits for all the DoF
    public float LIFT_MIN, LIFT_MAX, YAW_MIN, YAW_MAX, PITCH_MIN, PITCH_MAX;

    private void Start() {
        // Default values
        LIFT_MIN = 6;
        LIFT_MAX = 70;
        YAW_MIN = 0;
        YAW_MAX = 100;
        PITCH_MIN = 0;
        PITCH_MAX = 50;
    }

    /*
    private void Update() {
        // Test
        DeltaLift(Mathf.Sin(Time.time));
        //DeltaYaw(Mathf.Sin(Time.time));
        DeltaPitch(Mathf.Cos(Time.time));
    }
    */

    /* MiRo can feedback the positions of its DoF. */
    public float GetLift() {
        return neck.transform.localEulerAngles.y;
    }

    public float GetYaw() {
        return gimbal.transform.localEulerAngles.z;
    }

    public float GetPitch() {
        return head.transform.localEulerAngles.y;
    }

    // Return float array of current DoF positions
    public float[] GetCurrentAngles() {
        return new float[] { GetLift(), GetYaw(), GetPitch() };
    }

    // deltaAngles must be given as {lift, yaw, pitch}
    public void Rotate(float[] deltaAngles) {
        DeltaLift(deltaAngles[0]);
        DeltaYaw(deltaAngles[1]);
        DeltaPitch(deltaAngles[2]);
    }

    /* In reality, only changes in angles can be set */
    public void DeltaLift(float deg) {
        float newLiftAngle = GetLift() + deg;
        neck.transform.localEulerAngles = new Vector3(
            neck.transform.localEulerAngles.x,
            Mathf.Clamp(newLiftAngle, LIFT_MIN, LIFT_MAX), // Can't exceed limits
            neck.transform.localEulerAngles.z
        );
    }

    public void DeltaYaw(float deg) {
        float newYawAngle = GetYaw() + deg;

        gimbal.transform.localEulerAngles = new Vector3(
            gimbal.transform.localEulerAngles.x,
            gimbal.transform.localEulerAngles.y,
            Mathf.Clamp(newYawAngle, YAW_MIN, YAW_MAX) // Can't exceed limits
        );

    }

    public void DeltaPitch(float deg) {
        float newPitchAngle = GetPitch() + deg;

        head.transform.localEulerAngles = new Vector3(
            head.transform.localEulerAngles.x,
            Mathf.Clamp(newPitchAngle, PITCH_MIN, PITCH_MAX), // Can't exceed limits
            head.transform.localEulerAngles.z
        );
    }
}
