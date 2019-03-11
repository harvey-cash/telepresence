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
    public Camera leftCamera, rightCamera;
    private RenderTexture rendTex;

    private float timePoseHead;

    private VirtualMotors motors;
    public Transform headTransform;

    private void Awake() {
        if (!Config.USE_MIRO_SERVER) {
            Network.Join(this);
        }
        else {
            Destroy(this);
        }
    }

    private void Start() {
        motors = GetComponent<VirtualMotors>(); // affect the state of our robot model

        // Ensure cameras are disabled for manual rendering
        CameraSetup();

        // Record camera imagery repeatedly
        // Imagery and Pose are automatically posted together
        InvokeRepeating("TakeImagery", 1f, Config.ROBOT_FRAME_WAIT_MS / 1000f);
    }

    // Inverse-Kinematics the robot towards the closest matching head pose
    public override void ReceiveHeadPose(float timestamp, Pose headPose) {
        if (timestamp > timePoseHead) {
            timePoseHead = timestamp;

            // USE INVERSE KINEMATICS TO GET ANGLES
            float[] targetAngles = NeckKinematics.FindAngles(headPose);
            //Debug.Log("TARGET {lift: " + targetAngles[0] + ", yaw: " + targetAngles[1] + ", pitch: " + targetAngles[2] + "}");

            // MOVE ROBOT TOWARDS TARGET ANGLES
            float[] currentAngles = motors.GetCurrentAngles();
            //Debug.Log("CURRENT {lift: " + currentAngles[0] + ", yaw: " + currentAngles[1] + ", pitch: " + currentAngles[2] + "}");

            float[] deltaAngles = MotorController.DeltaDegrees(currentAngles, targetAngles);
            //Debug.Log("DELTA {lift: " + deltaAngles[0] + ", yaw: " + deltaAngles[1] + ", pitch: " + deltaAngles[2] + "}");

            motors.Rotate(deltaAngles);
            
        }        
    }

    // Disable cameras to avoid unecessary overhead, and create RenderTextures
    private void CameraSetup() {
        leftCamera.enabled = false;
        rightCamera.enabled = false;
        rendTex = new RenderTexture(Config.ROBOT_IMAGE_WIDTH, Config.ROBOT_IMAGE_HEIGHT, 24);
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

        PostImageryAndPose(left, right);
    }

    // Creating new Textures so rapidly leads to filling memory very quickly. 
    // Take care to ensure they are always eventually garbage collected.
    // 
    private byte[] GetCameraImage(Camera eyeCam) {
        Texture2D image = new Texture2D(Config.ROBOT_IMAGE_WIDTH, Config.ROBOT_IMAGE_HEIGHT, TextureFormat.RGB24, false);

        eyeCam.targetTexture = rendTex;
        eyeCam.Render();
        RenderTexture.active = rendTex;
        image.ReadPixels(new Rect(0, 0, Config.ROBOT_IMAGE_WIDTH, Config.ROBOT_IMAGE_HEIGHT), 0, 0);        
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
    private void PostImageryAndPose(byte[] left, byte[] right) {
        // Post images to Stitching Server
        System.Action<float, byte[], byte[], Pose> target = Network.Server.ReceiveImageryAndPose;
        //System.Action<float, byte[], Pose> target = Network.User.ReceiveImageryAndPose;

        // CREATE ROBOT POSE FROM NECK MODEL
        // Pose robotHeadPose = NeckKinematics.GetHeadPose(motors.GetCurrentAngles());

        // Unity child hierarchy provides us with a quick means of calculating forward kinematics
        Pose pose = new Pose(headTransform.position, headTransform.rotation);

        // Simulate network delay
        StartCoroutine(Network.Post(target, Time.time / 1000f, left, right, pose));
        //StartCoroutine(Network.Post(target, Time.time / 1000f, left, pose));
    }

    
}
