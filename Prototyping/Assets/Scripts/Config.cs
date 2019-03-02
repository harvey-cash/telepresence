using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Config
{
    // ~~~~~ NETWORK ~~~~~ //
    public static bool SIMULATE_DELAY = true; // Use network delay?
    public static bool RANDOM_RANGE = true; // If false, use max delay
    public const float MIN_NETWORK_DELAY_MS = 100f;
    public const float MAX_NETWORK_DELAY_MS = 200f;

    // ~~~~~ DECOUPLING ~~~~~ //

    public static bool DECOUPLE = true; // Use decoupling technique?
    public static bool CENTER_DISPLAY_ON_HEAD = true; // Center display position on head?

    // ~~~~~ HEAD ~~~~~~ //

    public const float POST_HEAD_POSE_MS = 20; // Post head pose to robot every x ms

    // ~~~~~ ROBOT ~~~~~ //
    
    public const int ROBOT_IMAGE_WIDTH = 1280, ROBOT_IMAGE_HEIGHT = 720; // Eye imagery from the robot
    public const float ROBOT_FRAME_WAIT_MS = 67f; // time in ms between frames. 67ms --> 15fps.
    public const float POST_ROBOT_POSE_MS = 20; // Post head pose to user every x ms
    public const float MAX_MOTOR_SPEED_DEG = 20; // degrees per second
}
