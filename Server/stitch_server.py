#
#   Stitching Server
#   Binds REP socket to tcp://*:5555
#

import time
import zmq
import cv2
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

print("~~ STITCHING SERVER V1.0 ~~")
print("~~ I'm doing good work ! ~~")
print("~~~~~~~   <(-_-)>   ~~~~~~~")

while True:
    #  Wait for next request from client
    message = socket.recv(0, True)

    # Split message into left and right images

    # Process

    #  Send reply back to client
    # ENCODE TO JPG BYTE STR FOR UNITY
    # cv2.imencode('.jpg', img)[1].tostring()
    socket.send(message)
