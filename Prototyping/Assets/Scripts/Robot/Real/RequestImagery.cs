using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RequestImagery : RunAbleThread {
    public bool sendData = true;
    public byte[] stitched;

    protected override void Run() {

        ForceDotNet.Force(); // prevent unity freeze after one use

        using (RequestSocket socket = new RequestSocket()) {
            socket.Connect(Config.GET_IMAGERY_IP);

            while (Running) {
                if (sendData) {
                    sendData = false;
                    stitched = Request(socket, "1");
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
}
