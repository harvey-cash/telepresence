using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* The VirtualDisplay is decoupled from the user's head pose
 * in order to account for discrepancies between user and robot.
 * Ideally, the display position should be accurate for the camera
 * imagery renderered to it.
 */
public abstract class VirtualDisplay : MonoBehaviour
{
    protected Renderer rend;
    protected Material surface;
    public abstract void SetRendAndSurface();

    protected User user;
    public void SetUser(User user) {
        this.user = user;
    }
    
    private Texture2D display;

    private float timePoseStabilise, timeImage;
    private Pose poseDelta;

    private void Start() {
        SetRendAndSurface();

        // Attach display to viewer
        if (!Config.DECOUPLE) {
            StartCoroutine(WaitThenAttach());
        }
    }

    // VR Prefab moves things around on the first frame, so if Coupled
    // We need to wait before setting the display as child
    private IEnumerator WaitThenAttach() {
        yield return new WaitForEndOfFrame();

        // In naive telepresence, we stick the display to the user's face
        transform.parent = user.viewer.GetComponent<Transform>();
        transform.localPosition = Vector3.zero;
        transform.localRotation = Quaternion.identity;
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
    public void ReceiveImageryAndPose(float timestamp, RenderTexture renderTexture, Pose pose) {
        // Ensure image chronology is maintained
        if (timestamp > timeImage) {
            timeImage = timestamp;

            Render(renderTexture, pose);
        }
    }

    private void Render(RenderTexture renderTexture, Pose pose) {
        // GARBAGE COLLECT FIRST, AS TEXTURES CAUSE MEMORY LEAKS IF SIMPLY
        // OVERWRITTEN.
        UnityEngine.Object.Destroy(display);

        display = new Texture2D(Config.ROBOT_IMAGE_WIDTH, Config.ROBOT_IMAGE_HEIGHT, TextureFormat.RGB24, false);
        RenderTexture.active = renderTexture;
        display.ReadPixels(new Rect(0, 0, Config.ROBOT_IMAGE_WIDTH, Config.ROBOT_IMAGE_HEIGHT), 0, 0); // Load JPEG into texture
        display.Apply();
        RenderTexture.active = null;

        surface.mainTexture = display;

        DestroyImmediate(renderTexture);

        // View decoupling
        if (Config.DECOUPLE) {
            UpdatePose(pose);
        }
    }

    // Update pose to most accurate representation, as frame has been rendered
    private void UpdatePose(Pose pose) {
        transform.position = pose.position;
        transform.rotation = pose.rotation;
    }
}
