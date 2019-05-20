using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* Provide head tracking data! */
public class VRHeadset : Viewer {

    void Awake() {
        if (!Config.USE_VR) {
            // Destroy(gameObject);
        }
        else {
            Network.Join(this);
        }
    }

    public override Pose GetHeadPose() {
        return new Pose(transform.position, transform.rotation);
    }
}
