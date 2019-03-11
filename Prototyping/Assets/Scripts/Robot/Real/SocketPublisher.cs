using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SocketPublisher : RunAbleThread
{

    public string targetAngles = "{lift: 0.0, yaw: 0.0, pitch: 0.0}";

    protected override void Run() {

        ForceDotNet.Force(); // prevent unity freeze after one use

        using (RequestSocket socket = new RequestSocket()) {
            socket.Connect(Config.POST_HEAD_IP);

            while (Running) {                
                Debug.Log(Send(socket, targetAngles));
            }
        }

        NetMQConfig.Cleanup(); // prevent unity freeze after one use
    }

    private string Send(RequestSocket socket, string send) {

        socket.SendFrame(send);
        Debug.Log("sent");

        // Wait for response
        string response = null;
        /*
        bool gotResponse = false;        
        while (Running) {
            gotResponse = socket.TryReceiveFrameString(out response);
            if (gotResponse) {
                break;
            }
        }
        */
        return response = socket.ReceiveFrameString();
    }
}
