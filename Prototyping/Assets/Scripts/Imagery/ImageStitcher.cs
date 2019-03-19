using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ImageStitcher
{
    private VirtualDisplay virtualDisplay;

    public ImageStitcher(VirtualDisplay virtualDisplay) {
        this.virtualDisplay = virtualDisplay;
    }

    // Send final image to the VirtualDisplay for Rendering
    public void StitchThenRender(float timestamp, byte[] left, byte[] right, Pose pose) {
        // For now, we simply render one of the eyes.
        virtualDisplay.ReceiveImageryAndPose(timestamp, left, pose);
    }
}
