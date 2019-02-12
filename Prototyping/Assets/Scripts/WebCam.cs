using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WebCam : MonoBehaviour {

    WebCamTexture feed;
    Material surface;

    [SerializeField]
    int dNum;

	// Use this for initialization
	void Start () {
        WebCamDevice[] devices = WebCamTexture.devices;
        /*
        for (int i = 0; i < devices.Length; i++) {
            Debug.Log(devices[i].name);
        }
        */

        feed = new WebCamTexture(devices[dNum].name, 1920, 1080, 60);
        surface = GetComponent<Renderer>().material;

        surface.mainTexture = feed;
        
        feed.Play();
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
