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
        Quaternion deltaRot = Quaternion.Euler(deltaDegrees * localRotationAxis);
        Quaternion proposedRot = deltaRot * transform.localRotation;

        float proposedDeg = Vector3.Dot(localRotationAxis, proposedRot.eulerAngles);

        // Limits
        if (proposedDeg >= 0 && proposedDeg <= maxAngle) {
            transform.localRotation = proposedRot;
        }
    }
}
