using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlatDisplay : VirtualDisplay
{
    public override void SetRendAndSurface() {
        rend = GetComponentInChildren<Renderer>();
        surface = GetComponentInChildren<Renderer>().material;
    }

    private void Awake() {
        if (Config.CURVED_DISPLAY) {
            Destroy(this.gameObject);
        }
        else {
            Network.Join(this);
        }
    }
}
