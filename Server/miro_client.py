#!/usr/bin/python

# MIRO client has two jobs:
# 1) Subscribe to headtracking data from Unity (and control motors suitably)
# 2) Publish stitched imagery and robot head data to Unity

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

#Generate a fake enum for joint arrays
tilt, lift, yaw, pitch = range(4)

class client:

	def __init__(self):

		# ~~~~~~~~~ LOCAL FIELDS ~~~~~~~~~ #

		# variables to store input data
		self.input_camera = [None, None] # Left and right imagery
		self.input_package = None # Sensor data
		# Joint angle targets
		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]

		# Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# ~~~~~~~~~ ROS SUBSCRIPTIONS AND PUBLISHERS ~~~~~~~~~ #

		# publish kinematic angles
		self.pub_kin = rospy.Publisher(topic_base + "control/kinematic_joints", JointState, queue_size=0)
		self.pub_cos = rospy.Publisher(topic_base + "control/cosmetic_joints", Float32MultiArray, queue_size=0)

		# subsribe to cameras
		self.sub_caml = rospy.Subscriber(topic_base + "sensors/caml/compressed",
							CompressedImage, self.callback_caml)
		self.sub_camr = rospy.Subscriber(topic_base + "sensors/camr/compressed",
							CompressedImage, self.callback_camr)
		# subscribe to sensors
		self.sub_package = rospy.Subscriber(topic_base + "sensors/package",
					miro.msg.sensors_package, self.callback_package)


		# ~~~~~~~~~ UNITY PyZMQ CONNECTIONS ~~~~~~~~~ #

		self.context = zmq.Context()
		# Publish imagery and robot head pose
		self.socket_pub = self.context.socket(zmq.PUB)
		self.socket_pub.bind("tcp://*:5555") # Send on this port
		# Subscribe to head tracking data
		self.socket_sub = self.context.socket(zmq.SUB)
		self.socket_sub.connect("tcp://localhost:5556") # Receive on this port
		## TODO: zmq.Poller() ?

		# Test controlling yaw on separate thread
		self.thread_control = Thread(target=self.yaw_thread)


    # ~~~~~~~~~ MOTOR FEEDBACK AND CONTROL ~~~~~~~~ #

	# Test controlling joints
	def yaw_thread(self):
		while True:
			angle = math.sin(time.time())
			print angle
			self.move_head(0, angle, 0);
			time.sleep(0.5)

	# Try to match user's head pose
	def move_head(self, liftAngle, yawAngle, pitchAngle):
		self.kin_joints.position[lift] = liftAngle
		self.kin_joints.position[yaw] = yawAngle
		self.kin_joints.position[pitch] = pitchAngle
		# move head to angles
		self.pub_kin.publish(self.kin_joints)

	# MIRO seems to handle moving to a specified joint position
	# so may not need these in the end
	def get_joint_angles(self):
		# update kinematic joints
		joints = self.input_package.kinematic_joints.position
		self.LiftMeasured.set_value(math.degrees(joints[1]))
		self.YawMeasured.set_value(math.degrees(joints[2]))
		self.PitchMeasured.set_value(math.degrees(joints[3]))


	# ~~~~~~~~~ IMAGERY AND POSE ~~~~~~~~ #



    # ~~~~~~~~~ CALLBACKS ~~~~~~~~~ #

	# All sensor data
	def callback_package(self, msg):
		self.input_package = msg

    # Left and right camera imagery
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

			except CvBridgeError as e:
				# swallow error, silently
				pass



# ~~~~~~~~~ MAIN PROGRAM EXECUTION ~~~~~~~~~ #
if __name__ == "__main__":
	client = client()
	rospy.init_node("client")

	client.thread_control.start()
