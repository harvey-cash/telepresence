using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class represents our simulated MiRo robot.
 * Should include a model of how MiRo's head can move, and a control system for
 * matching received head pose's as closely as possible.
 * Also needs to log eye camera imagery continually.
 */
public class VirtualRobot : TelepresenceRobot
{
    public const float CAMERA_FRAME_WAIT = 67f; // time in ms between frames. Roughly 15FPS
    public const int IMAGE_WIDTH = 100, IMAGE_HEIGHT = 100;

    public Camera leftCamera, rightCamera;
    private RenderTexture renderTexture;
    private Rect rect;
    Texture2D image;

    public override void ReceiveHeadPose(float timestamp, Pose headPose) {
        // MOVE TOWARDS MATCHING POSE
    }

    private void Start() {
        // Ensure cameras are disabled for manual rendering
        leftCamera.enabled = false;
        rightCamera.enabled = false;

        // Post camera imagery repeatedly
        InvokeRepeating("PostImagery", 1f, CAMERA_FRAME_WAIT / 1000f);
    }

    // Send head pose over the Network
    private void PostImagery() {
        Texture2D left = RTImage(leftCamera);
        Texture2D right = RTImage(rightCamera);

        System.Action<float, Texture2D, Texture2D> target = Network.User.ReceiveCameraImagery;

        StartCoroutine(Network.Post(target, Time.time / 1000f, left, right));
    }

    // ~~~~~~~ CODE BELOW ADAPTED FROM UNITY DOCUMENTATION ~~~~~~~~ //

    // Take a "screenshot" of a camera's Render Texture.
    Texture2D RTImage(Camera eyeCam) {
        // creates off-screen render texture that can rendered into
        rect = new Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
        renderTexture = new RenderTexture(IMAGE_WIDTH, IMAGE_HEIGHT, 24);
        image = new Texture2D(IMAGE_WIDTH, IMAGE_HEIGHT, TextureFormat.RGB24, false);

        eyeCam.targetTexture = renderTexture;
        // Render the camera's view.
        eyeCam.Render();
        RenderTexture.active = renderTexture;

        image.ReadPixels(rect, 0, 0);

        // reset active camera texture and render texture
        eyeCam.targetTexture = null;
        RenderTexture.active = null;

        return image;
    }
}
