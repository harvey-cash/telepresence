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
    public const int IMAGE_WIDTH = 1280, IMAGE_HEIGHT = 720;

    public Camera leftCamera, rightCamera;
    private RenderTexture rendTex;

    public override void ReceiveHeadPose(float timestamp, Pose headPose) {
        // MOVE TOWARDS MATCHING POSE
    }

    private void Start() {
        // Ensure cameras are disabled for manual rendering
        CameraSetup();

        // Post camera imagery repeatedly
        InvokeRepeating("TakeImagery", 1f, CAMERA_FRAME_WAIT / 1000f);
    }

    // Disable cameras to avoid unecessary overhead, and create RenderTextures
    private void CameraSetup() {
        leftCamera.enabled = false;
        rightCamera.enabled = false;
        rendTex = new RenderTexture(IMAGE_WIDTH, IMAGE_HEIGHT, 24);
    }

    // Simply call the RTImage coroutine
    private void TakeImagery() {
        StartCoroutine(RTImage());
    }

    // Take a "screenshot" of each camera's Render Texture.
    // We wait for end of frame as otherwise Unity throws errors
    WaitForEndOfFrame frameEnd = new WaitForEndOfFrame();
    private IEnumerator RTImage() {
        yield return frameEnd;
        Texture2D left = GetCameraImage(leftCamera);
        Texture2D right = GetCameraImage(rightCamera);

        PostImagery(left, right);
    }

    // Creating new Textures so rapidly leads to filling memory very quickly. 
    // Take care to ensure they are always eventually garbage collected.
    // 
    private Texture2D GetCameraImage(Camera eyeCam) {
        Texture2D image = new Texture2D(IMAGE_WIDTH, IMAGE_HEIGHT, TextureFormat.RGB24, false);

        eyeCam.targetTexture = rendTex;
        eyeCam.Render();
        RenderTexture.active = rendTex;
        image.ReadPixels(new Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT), 0, 0);
        image.Apply();

        // reset active camera texture
        // must set camera target to null, or else game view flashes
        eyeCam.targetTexture = null;
        RenderTexture.active = null;

        return image;
    }

    // Send camera imagery over the Network
    private void PostImagery(Texture2D left, Texture2D right) {
        System.Action<float, Texture2D, Texture2D> target = Network.User.ReceiveCameraImagery;
        StartCoroutine(Network.Post(target, Time.time / 1000f, left, right));
    }

    
}
