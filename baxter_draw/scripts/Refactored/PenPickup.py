#!/usr/bin/env python


from __future__ import division
import argparse
import cv2
import numpy
import os
import roslib
import rospy
import struct
import sys
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from baxter_interface import CHECK_VERSION, Gripper, Limb, RobotEnable, camera
from baxter_pykdl import baxter_kinematics
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

########------------------------------------------------------------------------#########
######## This script performs detection of a whitebaord marker pen		     	#########
######## using Baxter's cuff camera and attempts to have the robot grasp it. 	#########
########------------------------------------------------------------------------#########
######## Author: Samacharn Sankul												#########
########------------------------------------------------------------------------#########


class PenPickup:

	def __init__(self, limb):
		if limb != 'left' and limb != 'right':
			raise "limb should be 'left' or 'right'"
		
		rospy.init_node('pen_pickup')

		rs = RobotEnable(CHECK_VERSION)
		rs.state().enabled

		self.limb = limb
		self.arm = Limb(self.limb)
		self.ns = "ExternalTools/" + self.limb + "/PositionKinematicsNode/IKService"
		self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
		self.ikreq = SolvePositionIKRequest()
		self.hdr = Header(stamp=rospy.Time.now(), frame_id='base')

		self.gripper = Gripper(self.limb, CHECK_VERSION)

		self.bridge = CvBridge()
		self.cam_left = camera.CameraController('left_hand_camera')
		self.cam_right = camera.CameraController('right_hand_camera')
		self.image_sub = rospy.Subscriber('/cameras/' + self.limb + '_hand_camera/image', Image, self.callback, queue_size=1, buff_size=100)

		self.max_rad = 225
		self.accept_rad = int(round(self.max_rad * 0.22))

		init_left_arm_pos = PoseStamped(
			header=self.hdr,
			pose=Pose(
				position=Point(
					x= 0.6,
					y= 0.25,
					z= 0.1
				),
				orientation=Quaternion(
					x= 0,
					y= 1,
					z= 0,
					w= 0
				)
			)
		)

		init_right_arm_pos = PoseStamped(
			header=self.hdr,
			pose=Pose(
				position=Point(
					x= 0.6,
					y= -0.25,
					z= 0.1
				),
				orientation=Quaternion(
					x= 0,
					y= 1,
					z= 0,
					w= 0
				)
			)
		)
		
		if self.limb == 'left':
			self.__moveto__(init_left_arm_pos)
			self.cam_right.close()
			self.cam_left.open()
		else:
			self.__moveto__(init_right_arm_pos)
			self.cam_left.close()
			self.cam_right.open()


	def callback(self, data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data)

			h, w, d = frame.shape
			vary_h = int(round(h * 0.05))
			vary_w = int(round(w * 0.05))
			half_h = int(round(h / 2))
			half_w = int(round(w / 2))

			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# Smooth it, otherwise a lot of false circles may be detected
			blur = cv2.GaussianBlur(gray, (9, 9), 2, 2)

			circles = cv2.HoughCircles(blur, cv2.cv.CV_HOUGH_GRADIENT, 1.9, 15, self.max_rad, 10)

			if circles is not None:
				if self.limb == 'left':
					self.cam_left.close()
				else:
					self.cam_right.close()

				end_p_pose = self.arm.endpoint_pose()

				# Convert the (x, y) coordinates and radius of the circles to integers
				circles = numpy.round(circles[0, :]).astype('int')

				# Loop over the (x, y) coordinates and radius of the circles
				for (x, y, r) in circles:
					# Draw the circle in the output image, then draw a rectangle,
				    # corresponding to the center of the circle

					move_x = 0
					move_y = 0
					move_z = 0
					if (x < half_w - vary_w) or (x > half_w + vary_w):
						move_y = (half_w - x) / w * 0.05
					if (y < half_h - vary_h) and (y > half_h + vary_h):
						move_x = (half_h - y) / h * 0.05
					if r < self.accept_rad / 2:
						move_z = -0.03
					elif r < self.accept_rad:
						move_z = -0.01

					if move_x != 0 or move_y != 0 or move_z != 0:
						if self.limb == 'left':
							move_y *= -1.0

						move_pose_stamped = PoseStamped(
							header=self.hdr,
							pose=Pose(
								position=Point(
									x= end_p_pose['position'][0] + move_x,
									y= end_p_pose['position'][1] + move_y,
									z= end_p_pose['position'][2] + move_z
								),
								orientation=Quaternion(
									x= 0,
									y= 1,
									z= 0,
									w= 0
								)
							)
						)
						print 'original:', x, y, r
						print 'move:', move_x, move_y, move_z
						self.__moveto__(move_pose_stamped)

						if self.limb == 'left':
							self.cam_left.open()
						else:
							self.cam_right.open()
					else:
						self.__pickup__()

					break

			# Show the output image
			cv2.imshow('Camera', frame)

			cv2.waitKey(1)

		except CvBridgeError, e:
			print e


	def grab_pen(self):
		while not rospy.is_shutdown():
			rospy.spin()

		
	def __pickup__(self):
		end_p_pose = self.arm.endpoint_pose()
		move_pose_stamped = PoseStamped(
			header=self.hdr,
			pose=Pose(
				position=Point(
					x= end_p_pose['position'][0],
					y= end_p_pose['position'][1] + 0.028,
					z= end_p_pose['position'][2]
				),
				orientation=Quaternion(
					x= 0,
					y= 1,
					z= 0,
					w= 0
				)
			)
		)
		self.__moveto__(move_pose_stamped)

		end_p_pose = self.arm.endpoint_pose()
		move_pose_stamped = PoseStamped(
			header=self.hdr,
			pose=Pose(
				position=Point(
					x= end_p_pose['position'][0] - 0.02,
					y= end_p_pose['position'][1],
					z= end_p_pose['position'][2]
				),
				orientation=Quaternion(
					x= 0,
					y= 1,
					z= 0,
					w= 0
				)
			)
		)
		self.__moveto__(move_pose_stamped)

		rospy.signal_shutdown('shutting down')


	def __moveto__(self, pose):
		self.ikreq.pose_stamp = []
		self.ikreq.pose_stamp.append(pose)
		try:
			rospy.wait_for_service(self.ns, 5.0)
			resp = self.iksvc(self.ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return 1

		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)

		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
				self.ikreq.SEED_USER: 'User Provided Seed',
				self.ikreq.SEED_CURRENT: 'Current Joint Angles',
				self.ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
			}.get(resp_seeds[0], 'None')

			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			self.arm.move_to_joint_positions(limb_joints)
		else:
			print 'INVALID POSE - No Valid Joint Solution Found'


def main(args):
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
	parser.add_argument('-l', '--limb', choices=['left', 'right'], required=True, help="the limb to test")

	args = parser.parse_args(rospy.myargv()[1:])

	pp = PenPickup(args.limb)
	pp.grab_pen()

	print 'Finished pen pickup process'


if __name__ == '__main__':
	main(sys.argv)
	sys.exit()
