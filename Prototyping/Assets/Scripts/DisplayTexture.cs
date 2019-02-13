using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DisplayTexture : MonoBehaviour
{
    RenderTexture rendTex;
    Material surface;
    Renderer rend;

    public Camera left, right;

    // Use this for initialization
    void Start() {
        rend = GetComponent<Renderer>();
        surface = GetComponent<Renderer>().material;

        rendTex = new RenderTexture(1280, 720, 24, RenderTextureFormat.Default, RenderTextureReadWrite.Default);
        surface.mainTexture = rendTex;

        left.targetTexture = rendTex;
        //right.targetTexture = rendTex;
    }

    // Update is called once per frame
    void Update() {
    }
}
