using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class RequestImagery : RunAbleThread {
    public bool sendData = true;
    public byte[] stitched;
    public Quaternion poseRot;
    public byte[] messageBytes;

    protected override void Run() {

        ForceDotNet.Force(); // prevent unity freeze after one use

        using (RequestSocket socket = new RequestSocket()) {
            socket.Connect(Config.GET_IMAGERY_IP);

            while (Running) {
                if (sendData) {
                    sendData = false;
                    messageBytes = Request(socket, "1");
                    Unpack(messageBytes);
                }
            }
        }

        NetMQConfig.Cleanup(); // prevent unity freeze after one use
    }

    private byte[] Request(RequestSocket socket, string send) {

        socket.SendFrame(send);

        // Wait for response
        byte[] response = null;
        bool gotResponse = false;
        while (Running) {
            gotResponse = socket.TryReceiveFrameBytes(out response);
            if (gotResponse) {
                break;
            }
        }
        sendData = true;
        return response;
    }

    // Protocol bytes 0:1[int x] 2:x+2[image bytes] x+3:end[rotation]
    private void Unpack(byte[] message) {
        byte[] imageBytes = new byte[] { message[0], message[1] };
        int imageByteLength = BitConverter.ToInt16(imageBytes, 0);

        // Fill stitched with the correct bytes
        stitched = new byte[imageByteLength];
        Array.Copy(message, 2, stitched, 0, imageByteLength);

        int byteRotLength = message.Length - (2 + imageByteLength);
        byte[] byteRot = new byte[byteRotLength];
        Array.Copy(message, 2 + imageByteLength, byteRot, 0, byteRotLength);
        string stringRot = System.Text.Encoding.Default.GetString(byteRot);

        // Miro returns roll, -yaw, pitch
        Vector3 degRot = StringToVector3(stringRot);
        Quaternion pitch = Quaternion.AngleAxis(-degRot[2], Vector3.right);
        Quaternion yaw = Quaternion.AngleAxis(-degRot[1] - 32, Vector3.up);
        Quaternion roll = Quaternion.AngleAxis(degRot[0], Vector3.forward);

        poseRot = yaw * pitch;
    }

    // Parse string into Vector3
    public static Vector3 StringToVector3(string sVector) {
        // Remove the parentheses
        if (sVector.StartsWith("[") && sVector.EndsWith("]")) {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');

        // store as a Vector3
        Vector3 result = new Vector3(
            (float)(Mathf.Rad2Deg * double.Parse(sArray[0])),
            (float)(Mathf.Rad2Deg * double.Parse(sArray[1])),
            (float)(Mathf.Rad2Deg * double.Parse(sArray[2]))
        );

        return result;
    }
}
