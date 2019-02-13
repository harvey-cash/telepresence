using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotHead : MonoBehaviour
{
    public Transform playerHead;

    private Vector3 offset;

    // Start is called before the first frame update
    void Start()
    {
        offset = new Vector3(
            transform.position.x - playerHead.position.x,
            0,
            0
        );
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = offset + playerHead.position;
        transform.eulerAngles = new Vector3(
            playerHead.eulerAngles.x,
            playerHead.eulerAngles.y,
            transform.eulerAngles.z
        );
    }
}
