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
    public void StitchThenRender(float timestamp, Texture2D left, Texture2D right) {
        // For now, we simply render one of the eyes.
        virtualDisplay.Render(timestamp, left);
    }
}
