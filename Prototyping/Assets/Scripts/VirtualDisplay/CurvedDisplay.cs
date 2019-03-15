using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CurvedDisplay : VirtualDisplay
{
    public override void SetRendAndSurface() {
        rend = GetComponent<Renderer>();
        surface = GetComponent<Renderer>().material;
    }

    private void Awake() {
        if (!Config.CURVED_DISPLAY) {
            Destroy(this.gameObject);
        }
        else {
            Network.Join(this);
        }
    }
}
