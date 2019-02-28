using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class returns a change in angle, based on provided
 * current and target angles */
public static class MotorController
{
    const float MAX_SPEED = 10; // degrees per second

    public static float DeltaDegrees(float current, float target) {
        float maxSpeed = MAX_SPEED * Time.deltaTime;

        // Simplistic Proportional Movement over time
        float delta = target - current;
        return Mathf.Clamp(delta * Time.deltaTime, -maxSpeed, maxSpeed);
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
