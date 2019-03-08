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
		self.sub_caml = rospy.Subscriber(topic_base + "sensors/caml/compressed",
							CompressedImage, self.callback_caml)
				self.sub_camr = rospy.Subscriber(topic_base + "sensors/camr/compressed",
							CompressedImage, self.callback_camr)

		# get imagery
		GObject.timeout_add(20, self.update_images)

	def update_images(self):
		# for each camera
		for i in range(2):
			# get image
			image = self.input_camera[i]
			print type(image)
			# Post image here?
		return True

	# image callbacks
	def do_auto_camera_zoom(self, image_height):

			if image_height <= 240:
				self.gui_ResolutionSelection.set_active(2)
			elif image_height > 600:
				self.gui_ResolutionSelection.set_active(0)
			else:
				self.gui_ResolutionSelection.set_active(1)
			self.on_ResolutionSelection_changed()

		def do_auto_camera_res(self, shape):

			l = ['240x180', '320x180', '320x240', '480x360', '640x360', '960x720', '1280x720']
			s = str(shape[1]) + "x" + str(shape[0])
			for i in range(len(l)):
				if s == l[i]:
					self.gui_CapResolutionSelection.set_active(i)
					return

	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

		self.callback_cam(ros_image, 1)

	def callback_cam(self, ros_image, index):
			# silently (ish) handle corrupted JPEG frames
			try:
				# convert compressed ROS image to raw CV image
				image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

				# set camera zoom automatically if has not been set already
				if not self.auto_camera_zoom is None:
					h = self.auto_camera_zoom[0]
					dt = time.time() - self.auto_camera_zoom[1]
					if h == 0:
						# initial state, set from first received frame regardless
						self.do_auto_camera_zoom(image.shape[0])
						self.auto_camera_zoom = None
						# for initial frame, also set resolution selector
						self.do_auto_camera_res(image.shape)
					elif dt > 4.0:
						self.auto_camera_zoom = None
					elif abs(image.shape[0] - h) < 32:
						self.do_auto_camera_zoom(h)
						self.auto_camera_zoom = None

				# do zoom
				if self.camera_zoom == "0.5x":
					image = cv2.resize(image, (int(image.shape[1] * 0.5), int(image.shape[0] * 0.5)))
				elif self.camera_zoom == "2x":
					image = cv2.resize(image, (int(image.shape[1] * 2.0), int(image.shape[0] * 2.0)))

				# store image for display
				self.input_camera[index] = image

			except CvBridgeError as e:

				# swallow error, silently
				#print(e)
				pass
