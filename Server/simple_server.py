#!/usr/bin/python

import time
import os

import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import math
import miro2 as miro

import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")
print("~~ STITCHING SERVER V1.1 ~~")
print("~~ I'm doing good work ! ~~")
print("~~~~~~~   <(n_n)>   ~~~~~~~")

while True:
    #  Wait for next request from client
    message = socket.recv(0, True)
    b = bytearray()
    b.extend(message)

    # Split message into left and right images
    img = cv2.imdecode(np.array(b), 1);
    # left = main.get_image(0)
    # right = main.get_image(1)

    # Process
    # img = left # temporary

    #  Send reply back to client
    # ENCODE TO JPG BYTE STR FOR UNITY
    stitched = cv2.imencode('.jpg', img)[1].tostring()
    socket.send(stitched)
