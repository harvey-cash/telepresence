using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class is used to simulate Network latency.
 * Users and Robots communicate to each other through Posts,
 * which are subject to variable latency.
 */
public static class Network {

    private const float MIN_LATENCY = 20f, MAX_LATENCY = 200f;

    public static User User { private set; get; }
    public static TelepresenceRobot Robot { private set; get; }

    // Overloaded methods to allow any Robot or User to join
    public static void Join(User u) {
        User = u;
    }
    public static void Join(TelepresenceRobot r) {
        Robot = r;
    }

    // Poster provides the address (method) they wish to post to.
    public static IEnumerator Post(System.Action<float,Pose> callBack, float timestamp, Pose pose) {
        float latency = Random.Range(MIN_LATENCY, MAX_LATENCY);
        yield return new WaitForSeconds(latency / 1000f);

        callBack(timestamp, pose);
    }

    // Poster provides the address (method) they wish to post to.
    public static IEnumerator Post(System.Action<float, Texture2D, Texture2D> callBack, float timestamp, Texture2D left, Texture2D right) {
        float latency = Random.Range(MIN_LATENCY, MAX_LATENCY);
        yield return new WaitForSeconds(latency / 1000f);

        callBack(timestamp, left, right);
    }
}
