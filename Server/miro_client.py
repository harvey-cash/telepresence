#!/usr/bin/python

# MIRO client has two jobs:
# 1) Subscribe to headtracking data from Unity (and control motors suitably)
# 2) Publish stitched imagery and robot head data to Unity

import time
import sys
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
import ast
import struct

#Generate a fake enum for joint arrays
tilt, lift, yaw, pitch = range(4)

class client:

	def __init__(self):

		# ~~~~~~~~~ DEBUGGING ~~~~~~~~~ #

		# self.check = True

		# ~~~~~~~~~ LOCAL FIELDS ~~~~~~~~~ #
		self.run_threads = True

		# variables to store input data
		self.input_camera = [None, None] # Left and right imagery
		self.input_package = None # Sensor data
		# Joint angle targets
		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]

		# Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()

		# ~~~~~~~~~~~ KINEMATICS ~~~~~~~~~~~ #

		# create kc object with default (calibration) configuration
		# of joints (and zeroed pose of FOOT in WORLD)
		self.kc = miro.utils.kc_interf.kc_miro()

		# create objects in HEAD
		self.pos = miro.utils.get("LOC_NOSE_TIP_HEAD")
		self.vec = np.array([1.0, 0.0, 0.0])

		# transform to WORLD (note use of "Abs" and "Rel"
		# for positions and directions, respectively)
		self.posw = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.pos)
		self.vecw = self.kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.vec)

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

		# Receive head tracking and push imagery on separate threads
		self.thread_control = Thread(target=self.receive_head_tracking)
		self.thread_imagery = Thread(target=self.send_imagery_and_pose)


    # ~~~~~~~~~ MOTOR FEEDBACK AND CONTROL ~~~~~~~~ #

	# Listen for head tracking data
	def receive_head_tracking(self):
		print "Head tracking thread started..."

		# Subscribe to head tracking data
		self.socket_head_data = self.context.socket(zmq.REP)
		self.socket_head_data.bind("tcp://*:5555") # Receive on this port

		while self.run_threads:

			message = self.socket_head_data.recv(0, True)
			data = ast.literal_eval(message)

			self.move_head(data["lift"], data["yaw"], data["pitch"])

			self.socket_head_data.send("1")


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


	# update configuration based on data
	def forward_kinematics(self):
		# NB: the immobile joint "TILT" is always at the same
		# angle, "TILT_RAD_CALIB"
		joints = self.input_package.kinematic_joints.position
		kinematic_joints = np.array([miro.constants.TILT_RAD_CALIB, joints[1], joints[2], joints[3]])
		self.kc.setConfig(kinematic_joints)

		# transform to WORLD
		self.posw = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.pos)
		self.vecw = self.kc.changeFrameRel(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.vec)


	# ~~~~~~~~~ IMAGERY AND POSE ~~~~~~~~ #

	# Listen for imagery request
	def send_imagery_and_pose(self):
		print "Imagery thread started..."

		# Wait then send imagery
		self.socket_imagery = self.context.socket(zmq.REP)
		self.socket_imagery.bind("tcp://*:5556") # Receive on this port

		while self.run_threads:

			message = self.socket_imagery.recv(0, True)

			# Get left and right images
			left = self.input_camera[0]
			right = self.input_camera[1]
			# World position and rotation (of head?)
			posw = self.posw
			vecw = self.vecw

			# Stitch images using OpenCV

			# ENCODE TO JPG BYTE STR FOR UNITY
			stitched = bytearray(cv2.imencode('.jpg', left)[1].tostring())

			# Compose protocol
			# BYTE ORDER: [IMAGE LENGTH BYTES][IMAGE BYTES][POSE BYTES]
			imageLengthBytes = bytearray(struct.pack('h', len(stitched)))

			stringRot = np.array2string(vecw, separator=',')
			poseBytes = bytearray(stringRot)

			# Concatenate and send
			messageBytes = imageLengthBytes + stitched + poseBytes
			self.socket_imagery.send(messageBytes)



    # ~~~~~~~~~ CALLBACKS ~~~~~~~~~ #

	# Body pose
	def callback_pose(self, msg):
		self.body_pose = msg
		print msg

	# All sensor data
	def callback_package(self, msg):
		self.input_package = msg
		self.forward_kinematics()

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
	client.thread_imagery.start()

	print ("Stitching Server Started.")
