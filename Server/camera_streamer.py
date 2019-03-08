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


class cam_stream:

	def __init__(self):
		# variables to store input data
		self.input_camera = [None, None]
		self.t_input_camera = [[], []]

		self.camera_zoom = None
		self.auto_camera_zoom = [0, 0] # determine zoom from first received frame
		self.frame_params = [180, '180w', 15]
		self.meas_fps = ["", ""]

		#Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subsribe to cameras
		self.sub_caml = rospy.Subscriber(topic_base + "sensors/caml",
							CompressedImage, self.callback_caml)
		self.sub_camr = rospy.Subscriber(topic_base + "sensors/camr",
							CompressedImage, self.callback_camr)

		# get imagery
		self.update_images()

	def update_images(self):
		# for each camera
		for i in range(2):
			# get image
			image = self.input_camera[i]
			print type(image)
			# Post image here?
		return True


	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

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

if __name__ == "__main__":
	main = cam_stream()
	rospy.init_node("cam_stream")
