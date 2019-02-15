using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* Provide head tracking data! */
public class VRHeadset : Viewer {

    public override Pose GetHeadPose() {
        return new Pose(transform.position, transform.rotation);
    }
}
