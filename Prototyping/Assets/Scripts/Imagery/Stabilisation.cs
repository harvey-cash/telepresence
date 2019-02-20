using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Stabilisation
{
    private VirtualDisplay virtualDisplay;

    public Stabilisation(VirtualDisplay virtualDisplay) {
        this.virtualDisplay = virtualDisplay;
    }

    // Pass a delta Pose to the VirtualDisplay, for it to use in the
    // estimation of the Robot's position
    public void Stabilise (float timestamp, byte[] left, byte[] right) {
        // For now, we do no stabilisation
        virtualDisplay.Stabilisation(timestamp, new Pose(Vector3.zero, Quaternion.identity));
    }
}
