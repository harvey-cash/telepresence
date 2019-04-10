using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class returns a change in angle, based on provided
 * current and target angles */
public static class MotorController
{
    private static float[] accumError = new float[3];
    private static float accumLimit = 1;

    private static float kP = 0.8f, kI = 0.02f;

    public static float DeltaDegrees(int dof, float current, float target) {
        accumError[dof] = Mathf.Clamp(accumError[dof], accumLimit, accumLimit); // When past range of motion, don't increase massively

        float maxSpeed = Config.MAX_MOTOR_SPEED_DEG * Time.deltaTime;

        // Proportional, Integral Controller
        float error = target - current;
        accumError[dof] += error * Config.POST_HEAD_POSE_MS;

        float output = (error * kP) + (accumError[dof] * kI);

        return Mathf.Clamp(output, -maxSpeed, maxSpeed);
    }

    // Call for each given DoF (lift, yaw, pitch)
    public static float[] DeltaDegrees(float[] current, float[] target) {
        // Account for motor offsets
        target = new float[] { 0f, target[1] + 50, target[2] + 22 };

        float[] deltas = new float[current.Length];
        for (int i = 0; i < current.Length; i++) {
            deltas[i] = DeltaDegrees(i, current[i], target[i]);
        }
        return deltas;
    }
}
