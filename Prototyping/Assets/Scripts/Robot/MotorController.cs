using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class returns a change in angle, based on provided
 * current and target angles */
public static class MotorController
{

    public static float DeltaDegrees(float current, float target) {
        // Simplistic Proportional Movement over time
        float delta = target - current;
        return delta * Time.deltaTime;
    }

    // Call for each given DoF (lift, yaw, pitch)
    public static float[] DeltaDegrees(float[] current, float[] target) {
        float[] deltas = new float[current.Length];
        for (int i = 0; i < current.Length; i++) {
            deltas[i] = DeltaDegrees(current[i], target[i]);
        }
        return deltas;
    }
}
