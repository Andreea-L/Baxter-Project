#!/usr/bin/env python

import rospy
import numpy
import sys
import struct


from sensor_msgs.msg import Range
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from std_msgs.msg import Header

import baxter_interface
from baxter_pykdl import baxter_kinematics

########----------------------------------------------------------------#########
######## This script is responsible for executing the drawing, 		    #########
######## by transforming Cartesian poses into joint positions using IK. #########
########----------------------------------------------------------------#########
######## Author: Andreea Lutac											#########
########----------------------------------------------------------------#########




### CONSTANTS ###
Z_COORD = -0.0390009425941977
ROTATION = [-0.124120878028, 0.991890846532, 0.00478437869368,-0.0269010394494]

	


def main():

	### SETUP ###
	
	#rospy.init_node('baxter_draw')
	ns = "ExternalTools/right/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')

	arm = baxter_interface.Limb('right')
	joints = arm.joint_names()

	

	# Process file and create poses from the given coordinates
	poses = {}
	
	file = open("/home/level3_team/catkin_ws/src/level3_baxter/scripts/yobackFinalProcessed.txt")

	for line in file:
		tokens = line.split(" ")
		points.append([float(tokens[0]),float(tokens[1])])
		
		newPose1 = PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
							x= float(tokens[1]),
							y= float(tokens[0]),
							z= Z_COORD,
					),
					orientation=Quaternion(
							x= ROTATION[0],
							y= ROTATION[1],
							z= ROTATION[2],
							w= ROTATION[3],
					),
				),
			)
		poses.update({len(poses):newPose1})

		if len(tokens) == 5:	# line contains two (x,y) pairs, account for second one
			points.append([float(tokens[2]),float(tokens[3])])
			newPose2 = PoseStamped(
					header=hdr,
					pose=Pose(
						position=Point(
							x=float(tokens[3]),
							y= float(tokens[2]),
							z= Z_COORD,
						),
						orientation=Quaternion(
							x= ROTATION[0],
							y= ROTATION[1],
							z= ROTATION[2],
							w= ROTATION[3],
						),
					),
				)
			
			poses.update({len(poses):newPose2})


	# Add poses to IK service pose stamp list
	i = 0
	while (i<len(poses)):
		ikreq.pose_stamp.append(poses[i])
		i = i+1


	### INVERSE KINEMATICS SERVICE CALL ###
	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)		# Joint response from IK service
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 1

	resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
								resp.result_type)
	if (resp_seeds[0] != resp.RESULT_INVALID):
		seed_str = {
					ikreq.SEED_USER: 'User Provided Seed',
					ikreq.SEED_CURRENT: 'Current Joint Angles',
					ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
					}.get(resp_seeds[0], 'None')
		print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
			(seed_str,))


	### DRAWING ###
	for j in xrange(len(poses)):
			# Move to point
			limb_joints = dict(zip(resp.joints[j].name, resp.joints[j].position))
			print "Move to position: "+str(j)+" out of "+str(len(poses))
			arm.move_to_joint_positions(limb_joints)
			rospy.sleep(0.1)

			# Raise arm
			current_joints = arm.joint_angle("right_s1")
			print "Raising arm..."
			joint_command = {"right_s1": current_joints -0.07}
			arm.move_to_joint_positions(joint_command)


if __name__ == "__main__":
    main()
