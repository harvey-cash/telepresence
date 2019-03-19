using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Viewer : MonoBehaviour {    

    public abstract Pose GetHeadPose();

    private float waitThenSetSeconds = 5;
    public void SetDisplayPosition(System.Action<Vector3> callBack) {
        StartCoroutine(WaitThenGetPose(callBack));
    }

    private IEnumerator WaitThenGetPose(System.Action<Vector3> callBack) {
        yield return new WaitForSeconds(waitThenSetSeconds);
        callBack(GetHeadPose().position);
    }

}
