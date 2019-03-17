using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class is used to simulate Network latency.
 * Users and Robots communicate to each other through Posts,
 * which are subject to variable latency.
 */
public static class Network {

    public static User User { private set; get; }
    public static Viewer Viewer { private set; get; }
    public static VirtualDisplay Display { private set; get; }
    public static TelepresenceRobot Robot { private set; get; }
    public static StitchingServer Server { private set; get; }

    // Overloaded methods to allow any Robot, User, or Server to join
    public static void Join(User u) { User = u; }
    public static void Join(Viewer v) { Viewer = v; }
    public static void Join(VirtualDisplay d) { Display = d; }
    public static void Join(TelepresenceRobot r) { Robot = r; }
    public static void Join(StitchingServer s) { Server = s; }

    // Poster provides the address (method) they wish to post to.
    public static IEnumerator Post(System.Action<float,Pose> callBack, float timestamp, Pose pose) {
        if (Config.SIMULATE_DELAY)
            yield return new WaitForSeconds(NetworkDelayMS() / 1000f);
        else
            yield return new WaitForEndOfFrame();

        callBack(timestamp, pose);
    }

    // Poster provides the address (method) they wish to post to.
    public static IEnumerator Post(System.Action<float,byte[],byte[],Pose> callBack, float timestamp, byte[] left, byte[] right, Pose pose) {
        if (Config.SIMULATE_DELAY)
            yield return new WaitForSeconds(NetworkDelayMS() / 1000f);
        else
            yield return new WaitForEndOfFrame();

        callBack(timestamp, left, right, pose);
    }

    // Poster provides the address (method) they wish to post to.
    public static IEnumerator Post(System.Action<float, byte[], Pose> callBack, float timestamp, byte[] left, Pose pose) {
        if (Config.SIMULATE_DELAY)
            yield return new WaitForSeconds(NetworkDelayMS() / 1000f);
        else
            yield return new WaitForEndOfFrame();

        callBack(timestamp, left, pose);
    }

    // Poster provides the address (method) they wish to post to.
    public static IEnumerator Post(System.Action<float, float> callBack, float leftWheel, float rightWheel) {
        if (Config.SIMULATE_DELAY)
            yield return new WaitForSeconds(NetworkDelayMS() / 1000f);
        else
            yield return new WaitForEndOfFrame();

        callBack(leftWheel, rightWheel);
    }

    // Get a network delay in ms
    private static float NetworkDelayMS() {
        // If simulating noise, then vary between min and max
        // Else, simply offset by max
        if (Config.RANDOM_RANGE)
            return Random.Range(Config.MIN_NETWORK_DELAY_MS, Config.MAX_NETWORK_DELAY_MS);
        else
            return Config.MAX_NETWORK_DELAY_MS;
    }
}
