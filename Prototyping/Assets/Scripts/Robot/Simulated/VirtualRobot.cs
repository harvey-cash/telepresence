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
    public Transform body;

    public Camera centerCamera; //, rightCamera;

    private float timePoseHead;

    private VirtualMotors motors;
    public Transform eyeTransform;

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
            Debug.Log("TARGET {lift: " + targetAngles[0] + ", yaw: " + targetAngles[1] + ", pitch: " + targetAngles[2] + "}");

            // MOVE ROBOT TOWARDS TARGET ANGLES
            float[] currentAngles = motors.GetCurrentAngles();
            Debug.Log("CURRENT {lift: " + currentAngles[0] + ", yaw: " + currentAngles[1] + ", pitch: " + currentAngles[2] + "}");

            float[] deltaAngles = MotorController.DeltaDegrees(currentAngles, targetAngles);
            Debug.Log("DELTA {lift: " + deltaAngles[0] + ", yaw: " + deltaAngles[1] + ", pitch: " + deltaAngles[2] + "}");

            motors.Rotate(deltaAngles);
            
        }        
    }

    // Disable cameras to avoid unecessary overhead, and create RenderTextures
    private void CameraSetup() {
        centerCamera.enabled = false;
        //rightCamera.enabled = false;
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
        RenderTexture rend = GetCameraImage(centerCamera);

        PostImageryAndPose(rend);
    }

    // Creating new Textures so rapidly leads to filling memory very quickly. 
    // Take care to ensure they are always eventually garbage collected.
    // 
    private RenderTexture GetCameraImage(Camera eyeCam) {
        RenderTexture renderTexture = new RenderTexture(Config.ROBOT_IMAGE_WIDTH, Config.ROBOT_IMAGE_HEIGHT, 24);
        eyeCam.targetTexture = renderTexture;
        eyeCam.Render();             

        // reset active camera texture
        // must set camera target to null, or else game view flashes
        eyeCam.targetTexture = null;
        RenderTexture.active = null;

        return renderTexture;
    }

    // Send camera imagery over the Network
    // Encoded as JPEG
    private void PostImageryAndPose(RenderTexture renderTexture) {
        // Post images to Stitching Server
        // System.Action<float, byte[], byte[], Pose> target = Network.Server.ReceiveImageryAndPose;
        System.Action<float, RenderTexture, Pose> target = Network.User.ReceiveImageryAndPose;

        // CREATE ROBOT POSE FROM NECK MODEL
        // Pose robotHeadPose = NeckKinematics.GetHeadPose(motors.GetCurrentAngles());

        // Unity child hierarchy provides us with a quick means of calculating forward kinematics
        Vector3 localPos = eyeTransform.position - body.position;
        Quaternion localRot = eyeTransform.rotation; //  Quaternion.FromToRotation(body.forward, eyeTransform.forward);
        Pose pose = new Pose(localPos, localRot);
        

        // Simulate network delay
        // StartCoroutine(Network.Post(target, Time.time / 1000f, left, right, pose));
        StartCoroutine(Network.Post(target, Time.time / 1000f, renderTexture, pose));
    }

    // User sends velocities for each wheel, and we move as a result
    public override void WheelVel(float left, float right) {
        float wheelDist = 0.1f;
        float diff = left - right;
        Vector3 centre = body.position + body.right * diff * wheelDist;

        body.transform.RotateAround(centre, Vector3.up, diff * Time.deltaTime * 50);
        float avg = (left + right) / 2;
        body.transform.localPosition += body.transform.forward * avg * Time.deltaTime * 0.5f;
    }
}
