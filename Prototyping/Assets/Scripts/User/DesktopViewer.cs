using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Characters.FirstPerson;

public class DesktopViewer : Viewer
{

    public override Pose GetHeadPose() {
        return new Pose(gameObject.transform.position, gameObject.transform.rotation);
    }

    private void Awake() {
        if (Config.USE_VR) {
            Destroy(this.gameObject);
        }
        else {
            Network.Join(this);
        }
    }

    // Update is called once per frame
    Vector2 rotation = new Vector2(0, 0);
    public float speed = 3;

    void Update() {
        float h = Input.GetAxis("RightStickHorizontal") * 0.1f;
        float v = Input.GetAxis("RightStickVertical") * 0.1f;

        rotation.y += Input.GetAxis("Mouse X") + h;
        rotation.x += -Input.GetAxis("Mouse Y") + v;
        transform.eulerAngles = (Vector2)rotation * speed;
    }
}
