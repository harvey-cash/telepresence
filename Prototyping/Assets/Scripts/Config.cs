using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Config
{
    // ~~~~~ DECOUPLING ~~~~~ //

    public static bool USE_VR = true; // VR or desktop?
    //public static bool DECOUPLE = true && USE_VR; // Use decoupling technique? (only for VR!)
    public static bool CENTER_DISPLAY_ON_HEAD = false; // Center display position on head?

    // ~~~~~ NETWORK ~~~~~ //

    public const string POST_HEAD_IP = "tcp://192.168.11.208:5555"; //5
    public const string GET_IMAGERY_IP = "tcp://192.168.11.208:5556"; //6

    // ~~~~~ SIMULATED NETWORK ~~~~~ //

    //public static bool SIMULATE_DELAY = false; // Use network delay?
    public static bool RANDOM_RANGE = false; // If false, use max delay
    public const float MIN_NETWORK_DELAY_MS = 200f;
    public const float MAX_NETWORK_DELAY_MS = 50f;

    // ~~~~~ DISPLAY ~~~~~ //

    public static bool CURVED_DISPLAY = false;

    // ~~~~~ HEAD ~~~~~~ //

    public static readonly float[] MIRO_CALIBRATE_DEG = new float[] { 34f, 0f, 0f }; // Directly forwards
    public static readonly float[] VR_CALIBRATE_DEG = new float[] { 0f, 0f, 0f }; // Directly forwards
    public const float POST_HEAD_POSE_MS = 20; // Post head pose to robot every x ms

    // ~~~~~ ROBOT ~~~~~ //

    public static bool USE_MIRO_SERVER = false;
    
    public const int ROBOT_IMAGE_WIDTH = 1280, ROBOT_IMAGE_HEIGHT = 720; // Eye imagery from the robot
    public const float ROBOT_FRAME_WAIT_MS = 67f; // time in ms between frames. 67ms --> 15fps.
    public const float MIRO_CHECK_FRAME_WAIT_MS = 20f;
    public const float POST_ROBOT_POSE_MS = 20; // Post head pose to user every x ms
    public const float MAX_MOTOR_SPEED_DEG = 500; // degrees per second
}
