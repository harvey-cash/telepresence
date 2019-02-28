using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Motor : MonoBehaviour
{

    public float maxAngle;
    public Vector3 localRotationAxis;

    public float CurrentAngle { get { return Vector3.Dot(localRotationAxis, transform.localEulerAngles); } }

    // Rotate as told, within the angle limits of the motor
    public void Rotate(float deltaDegrees) {
        Quaternion rotation = Quaternion.Euler(deltaDegrees * localRotationAxis);
        transform.localRotation = rotation * transform.localRotation;

        /*
        if (CurrentAngle + deltaDegrees > 0 && CurrentAngle + deltaDegrees < maxAngle) {

            transform.localEulerAngles = new Vector3(
                    transform.localEulerAngles.x,
                    transform.localEulerAngles.y,
                    CurrentAngle + deltaDegrees
            );
        }
        */
    }
}
