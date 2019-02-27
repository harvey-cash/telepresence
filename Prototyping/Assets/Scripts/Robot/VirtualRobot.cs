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
    public float CAMERA_FRAME_WAIT = 67f; // time in ms between frames. 67ms --> 15fps.
    public float POSE_WAIT = 20f;

    public Camera leftCamera, rightCamera;
    private RenderTexture rendTex;

    private float timePoseHead;

    // Inverse-Kinematics the robot towards the closest matching head pose
    public override void ReceiveHeadPose(float timestamp, Pose headPose) {
        if (timestamp > timePoseHead) {
            timePoseHead = timestamp;
            
            // USE INVERSE KINEMATICS TO MOVE TO HEAD POSE

        }        
    }

    private void Start() {
        // Ensure cameras are disabled for manual rendering
        CameraSetup();

        // Post camera imagery repeatedly
        InvokeRepeating("TakeImagery", 1f, CAMERA_FRAME_WAIT / 1000f);
        // Post head pose repeatedly
        InvokeRepeating("PostHeadPose", 0, POSE_WAIT / 1000f);
    }

    // Disable cameras to avoid unecessary overhead, and create RenderTextures
    private void CameraSetup() {
        leftCamera.enabled = false;
        rightCamera.enabled = false;
        rendTex = new RenderTexture(Config.IMAGE_WIDTH, Config.IMAGE_HEIGHT, 24);
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
        byte[] left = GetCameraImage(leftCamera);
        byte[] right = GetCameraImage(rightCamera);

        PostImagery(left, right);
    }

    // Creating new Textures so rapidly leads to filling memory very quickly. 
    // Take care to ensure they are always eventually garbage collected.
    // 
    private byte[] GetCameraImage(Camera eyeCam) {
        Texture2D image = new Texture2D(Config.IMAGE_WIDTH, Config.IMAGE_HEIGHT, TextureFormat.RGB24, false);

        eyeCam.targetTexture = rendTex;
        eyeCam.Render();
        RenderTexture.active = rendTex;
        image.ReadPixels(new Rect(0, 0, Config.IMAGE_WIDTH, Config.IMAGE_HEIGHT), 0, 0);        
        image.Apply();        

        // reset active camera texture
        // must set camera target to null, or else game view flashes
        eyeCam.targetTexture = null;
        RenderTexture.active = null;

        byte[] jpeg = ImageConversion.EncodeToJPG(image);

        // Ensure garbage is collected
        Destroy(image);

        return jpeg;
    }

    // Send camera imagery over the Network
    // Encoded as JPEG
    private void PostImagery(byte[] left, byte[] right) {
        System.Action<float, byte[], byte[]> target = Network.User.ReceiveCameraImagery;
        StartCoroutine(Network.Post(target, Time.time / 1000f, left, right));
    }

    private void PostHeadPose() {
        System.Action<float, Pose> target = Network.User.ReceiveRobotPose;
        // let's cheat, for now!
        StartCoroutine(Network.Post(target, Time.time / 1000f, new Pose(transform.position, transform.rotation)));
    }

    
}
