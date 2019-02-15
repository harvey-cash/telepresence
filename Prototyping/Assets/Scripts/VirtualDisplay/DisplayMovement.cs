using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DisplayMovement : MonoBehaviour
{
    public Transform playerHead, robotHead;

    // Use this for initialization
    void Start() {
    }

    // Update is called once per frame
    void Update() {
        transform.position = playerHead.position;
        transform.rotation = robotHead.rotation;
    }
}
