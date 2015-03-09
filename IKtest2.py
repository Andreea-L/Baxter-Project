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

### CONSTANTS ###
Z_COORD = -0.041109425941977
ROTATION = [-0.124120878028, 0.991890846532, 0.00478437869368,-0.0269010394494]
DISTANCE = 0.153999999166

### GLOBALS ###
inPosition = False;
joints = []
current_pos = []
arm = None
sub = None


def checkDistance(data):
	global sub
	#rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)
	print data.range
	# print DISTANCE - 0.005
	# print DISTANCE + 0.005
	if data.range >= DISTANCE-0.005 and data.range <= DISTANCE+0.005:
	 	print "In range."
	else:
		adjustZ(-0.007)
	sub.unregister()

def getDistance():
	global sub
	print "Commencing listen."
	sub = rospy.Subscriber("/robot/range/right_hand_range/state", Range, checkDistance)
	#rospy.spin()


def adjustZ(offset):
	global current_pos, inPosition
	arm = baxter_interface.Limb('right')
	new_pos = [current_pos[0],current_pos[1],Z_COORD+offset]

	print new_pos
	joint_positions = dict(zip(joints,new_pos))

	arm.move_to_joint_positions(joint_positions)
	current_pos = new_pos
	inPosition = True;

# def raise_pen():
# 	global current_pos, inPosition
# 	arm = baxter_interface.Limb('right')
# 	raised_pos = [current_pos[0],current_pos[1],Z_COORD+0.05]

# 	print current_pos
# 	joint_positions = dict(zip(joints,raised_pos))

# 	arm.move_to_joint_positions(joint_positions)
# 	current_pos = raised_pos	
	

### EXECUTE MOVEMENT AND RECURSIVELY SPLIT FAILING SEGMENTS ###
def recursive_drawing(start_position, end_position, rec_depth):
	global current_pos, inPosition
	arm = baxter_interface.Limb('right')
	kin = baxter_kinematics('right')
	result = kin.inverse_kinematics(end_position,ROTATION)
	print result
	if rec_depth > 10:
		print "Recursing limit reached."
		return 0
	if result == None:
		mid_position = [(start_position[0]+end_position[0])/2,(start_position[1]+end_position[1])/2,Z_COORD]
		print "Splitting at: "+str(mid_position)
		recursive_drawing(start_position,mid_position,rec_depth+1)
		recursive_drawing(mid_position,end_position,rec_depth+1)
	else:
		print "Solution found"
		joint_positions = dict(zip(joints,result.tolist()))
		#print result
		arm.move_to_joint_positions(joint_positions)
		#rospy.sleep(0.1)
		# print "Actual z position: "+str(kin.forward_position_kinematics()[2])
		# print "Expected z position: "+str(Z_COORD)
		# print "Offset: "+str(kin.forward_position_kinematics()[2]-Z_COORD)
		current_pos = end_position
		inPosition = True	


def main():

	### SETUP ###
	global joints, arm, current_pos, inPosition
	rospy.init_node('baxter_draw')
	ns = "ExternalTools/right/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	arm = baxter_interface.Limb('right')
	joints = arm.joint_names()

	print '*** Baxter PyKDL Kinematics ***\n'
	
	
	file = open("/home/level3_team/catkin_ws/src/level3_baxter/scripts/scaledLinesTEST.txt")
	#print file
	current = arm.endpoint_pose()
	#print current
	current_pos = [current['position'].x,current['position'].y,current['position'].z]
	#print current_pos
	#recursive_drawing(current_pos,[0.066917404475,-0.343215954863,Z_COORD])
	poses = {}
	for line in file:
		tokens = line.split(" ")
		print tokens
		newPose1 = PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
							x= float(tokens[1]),
							y= float(tokens[0]),
							z= Z_COORD,
					),
					orientation=Quaternion(
							x= -0.124120878028,
							y= 0.991890846532,
							z= 0.00478437869368,
							w= -0.0269010394494,
					),
				),
			)
		newPose2 = PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
						x=float(tokens[3]),
						y= float(tokens[2]),
						z= Z_COORD,
					),
					orientation=Quaternion(
						x= -0.124120878028,
						y= 0.991890846532,
						z= 0.00478437869368,
						w= -0.0269010394494,
					),
				),
			)
		poses.update({len(poses):newPose1})
		poses.update({len(poses):newPose2})

	i = 0
	while (i<len(poses)):
		ikreq.pose_stamp.append(poses[i])
		i = i+1




	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 1

	print resp

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

	for j in xrange(len(poses)):
			limb_joints = dict(zip(resp.joints[j].name, resp.joints[j].position))
			arm.move_to_joint_positions(limb_joints)
			rospy.sleep(0.7)
			current_joints = arm.joint_angle("right_s1")
			print "Current shoulder joint angle: "+str(current_joints)
			joint_command = {"right_s1": current_joints -0.2}
			arm.move_to_joint_positions(joint_command)
			rospy.sleep(0.7)
			print "Current shoulder joint angle: "+str(arm.joint_angle("right_s1"))
	# ### PROCESS FILE ###
	# for line in file:
	# 	print "Processing file"
	# 	tokens = line.split(" ")
	# 	if len(tokens) < 4:
	# 		return
	# 	#print "TOKENS: "+tokens[0]+" "+tokens[1]+" "+tokens[2]+" "+tokens[3]+"\n"
	# 	print "Processing: "+tokens[0]+" "+tokens[1]+"\n"
	# 	pos1 = [float(tokens[1]),float(tokens[0]), Z_COORD]

		

	# 	# dist = ((pos1[0]-current_pos[0]) ** 2 + (pos1[1]-current_pos[1]) ** 2) ** 0.5
	# 	# print "Distance: "+str(dist)+"Is in position: "+str(inPosition)
	# 	# if dist>0.1 and inPosition == True:
	# 	# 	print "Distance: "+str(dist)
	# 	# 	print "Raising pen..."
		
	# 		#adjustZ(0.015)

	# 	#print "Distance: "+str(dist)


	# 	pos2 = [float(tokens[3]),float(tokens[2]), Z_COORD]
	# 	recursive_drawing(current_pos,pos1,1)
	# 	print "Processing: "+tokens[2]+" "+tokens[3]+"\n"
	# 	current_joints = arm.joint_angle("right_s1")

	# 	print "Current shoulder joint angle: "+str(current_joints)
	# 	joint_command = {"right_s1": current_joints -0.2}
	# 	arm.move_to_joint_positions(joint_command)
	# 	recursive_drawing(current_pos,pos2,1)
	# 	#getDistance()

	# file.close()


if __name__ == "__main__":
    main()
