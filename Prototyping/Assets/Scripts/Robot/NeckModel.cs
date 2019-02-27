using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/* This class represents the hierarchical nature
 * of a robot's head and neck physiology.
 */
public class NeckModel {

}

/* Each node rotates around its origin, in a single axis of rotation
 * Each node has a length, atop which its child node originates
 */
class Node {
    public Node parent;
    public Vector3 localRotAxis;
    public float angle, length;

    public Node(Node parent, Vector3 localRotAxis, float angle, float length) {
        this.parent = parent;
        this.localRotAxis = localRotAxis;
        this.length = length;
    }
}