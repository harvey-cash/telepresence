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
     
    public Motor lift, yaw, pitch;
    
    /*
    private void Update() {
        // Test
        DeltaLift(Mathf.Sin(Time.time));
        DeltaYaw(Mathf.Sin(Time.time));
        DeltaPitch(Mathf.Cos(Time.time));
    }
    */

    // Return float array of current DoF positions
    public float[] GetCurrentAngles() {
        return new float[] { 
            lift.CurrentAngle,
            yaw.CurrentAngle,
            pitch.CurrentAngle
        };
    }

    // deltaAngles must be given as {lift, yaw, pitch}
    public void Rotate(float[] deltaAngles) {
        lift.Rotate(deltaAngles[0]);
        yaw.Rotate(deltaAngles[1]);
        pitch.Rotate(deltaAngles[2]);
    }

}
