using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SocketPublisher : RunAbleThread
{

    protected override void Run() {

        ForceDotNet.Force(); // this line is needed to prevent unity freeze after one use, not sure why yet

        using (PublisherSocket socketPub = new PublisherSocket()) {
            //client.Connect("tcp://localhost:5555");
            socketPub.Bind("tcp://192.168.11.165:5555"); // LAPTOP!

            while (Running) {

                Publish(socketPub);

            }
        }

        NetMQConfig.Cleanup(); // this line is needed to prevent unity freeze after one use, not sure why yet
    }

    private void Publish(PublisherSocket socketPub) {

        NetMQMessage netMQMessage = new NetMQMessage();
        netMQMessage.Append("LIFT");
        netMQMessage.Append("hello_harvey");
        socketPub.TrySendMultipartMessage(netMQMessage);
        Debug.Log("Hello!");
    }
}
