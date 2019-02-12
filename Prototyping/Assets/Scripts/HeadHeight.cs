using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HeadHeight : MonoBehaviour {

    public Transform head;
    private float x, z;

	// Use this for initialization
	void Start () {
        x = transform.position.x;
        z = transform.position.z;
	}
	
	// Update is called once per frame
	void Update () {
        transform.position = new Vector3(x, head.position.y, z);
	}
}
