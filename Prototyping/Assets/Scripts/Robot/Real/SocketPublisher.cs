using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SocketPublisher : RunAbleThread
{

    protected override void Run() {

        ForceDotNet.Force(); // prevent unity freeze after one use

        using (RequestSocket socketPub = new RequestSocket()) {
            //client
            socketPub.Connect("tcp://192.168.11.165:5556"); // LAPTOP!

            while (Running) {

                Debug.Log(Send(socketPub, "Here."));

            }
        }

        NetMQConfig.Cleanup(); // prevent unity freeze after one use
    }

    private string Send(RequestSocket socket, string send) {

        socket.SendFrame(send);

        // Wait for response
        string response = null;
        bool gotResponse = false;
        while (Running) {
            gotResponse = socket.TryReceiveFrameString(out response);
            if (gotResponse) {
                break;
            }
        }
        return response;
    }
}
