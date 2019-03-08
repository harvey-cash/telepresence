#!/usr/bin/python

import time
import os
from threading import Thread

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

class cam_stream:

	def __init__(self):
		# variables to store input data
		self.input_camera = [None, None]
		#Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subsribe to cameras
		self.sub_caml = rospy.Subscriber(topic_base + "sensors/caml",
							CompressedImage, self.callback_caml)
		self.sub_camr = rospy.Subscriber(topic_base + "sensors/camr",
							CompressedImage, self.callback_camr)


    # GET IMAGE!
	def get_image(self, i):
		return self.input_camera[i]

	def callback_caml(self, ros_image):
		print "left eye"
		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):
		print "right eye"
		self.callback_cam(ros_image, 1)

	def callback_cam(self, ros_image, index):
			# silently (ish) handle corrupted JPEG frames
			try:
				# convert compressed ROS image to raw CV image
				image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

				# store image for display
				self.input_camera[index] = image
				cv2.imwrite("./img.jpg", image)

			except CvBridgeError as e:

				# swallow error, silently
				#print(e)
				pass


################################################################
## MAIN
def stitch(main):
	context = zmq.Context()
	socket = context.socket(zmq.REP)
	socket.bind("tcp://*:5555")
	print("~~ STITCHING SERVER V1.1 ~~")
	print("~~ I'm doing good work ! ~~")
	print("~~~~~~~   <(n_n)>   ~~~~~~~")

	while True:
		#  Wait for next request from client
		message = socket.recv(0, True)
		print "Received Request"
        # b = bytearray()
        # b.extend(message)

        # Split message into left and right images
        # img = cv2.imdecode(np.array(b), 1);
		left = main.get_image(0)
		right = main.get_image(1)

        # Process
        img = left # temporary

        #  Send reply back to client
        # ENCODE TO JPG BYTE STR FOR UNITY
        stitched = cv2.imencode('.jpg', img)[1].tostring()
        # socket.send(stitched)


if __name__ == "__main__":
	main = cam_stream()
	rospy.init_node("cam_stream")

	thread = Thread(target=stitch, args=(main,))
	thread.start()
