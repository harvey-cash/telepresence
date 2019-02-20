using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* The VirtualDisplay is decoupled from the user's head pose
 * in order to account for discrepancies between user and robot.
 * Ideally, the display position should be accurate for the camera
 * imagery renderered to it.
 */
public class VirtualDisplay : MonoBehaviour
{

    protected User user;
    public void SetUser(User user) {
        this.user = user;
    }

    private Renderer rend;
    private Material surface;
    private Texture2D display;

    private float timePoseRobot, timePoseStabilise, timeImage;
    private Pose poseRobot, poseDelta;

    private void Start() {
        rend = GetComponent<Renderer>();
        surface = GetComponent<Renderer>().material;
    }

    // In naive telepresence, we stick the display to the user's face
    private void Update() {
        if (!Config.DECOUPLE) {
            Pose viewerPose = user.viewer.GetHeadPose();
            transform.position = viewerPose.position;
            transform.rotation = viewerPose.rotation;
        }
    }

    // Received from Robot via Network and User
    public void ReceivePose(float timestamp, Pose pose) {
        // If pose is more recent than stored pose, update
        if (timestamp > timePoseRobot) {
            poseRobot = pose;
            timePoseRobot = timestamp;
        }
    }

    // Received from Stabilisation
    // Pose here is a difference to be applied to the robot pose
    public void Stabilisation(float timestamp, Pose deltaPose) {
        // If imagery for stabilisation is more recent than stored, update
        if (timestamp > timePoseStabilise) {
            poseDelta = deltaPose;
            timePoseStabilise = timestamp;
        }
    }

    // Combined imagery received from ImageStitcher
    public void Render(float timestamp, byte[] imagery) {
        // Ensure image chronology is maintained
        if (timestamp > timeImage) {
            timeImage = timestamp;

            // GARBAGE COLLECT FIRST, AS TEXTURES CAUSE MEMORY LEAKS IF SIMPLY
            // OVERWRITTEN.
            UnityEngine.Object.Destroy(display);

            display = new Texture2D(Config.IMAGE_WIDTH, Config.IMAGE_HEIGHT, TextureFormat.RGB24, false);
            display.LoadImage(imagery); // Load JPEG into texture
            surface.mainTexture = display;

            // View decoupling
            UpdatePose();
        }

        
    }

    // Update pose to most accurate representation, as frame has been rendered
    private void UpdatePose() {
        if (Config.DECOUPLE) {
            transform.position = poseRobot.position + poseDelta.position;
            transform.rotation = poseRobot.rotation; // ignore delta for now
        }
    }
}
