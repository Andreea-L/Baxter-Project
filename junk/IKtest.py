#!/usr/bin/env python

import rospy
import numpy
import sys


from sensor_msgs.msg import Range
import std_msgs.msg

import baxter_interface
from baxter_pykdl import baxter_kinematics

### CONSTANTS ###
Z_COORD = -0.042625941977
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
	
	arm = baxter_interface.Limb('right')
	joints = arm.joint_names()

	print '*** Baxter PyKDL Kinematics ***\n'
	
	
	file = open("/home/level3_team/catkin_ws/src/level3_baxter/scripts/linesScaled2.txt")
	current = arm.endpoint_pose()
	#print current
	current_pos = [current['position'].x,current['position'].y,current['position'].z]
	#print current_pos
	#recursive_drawing(current_pos,[0.066917404475,-0.343215954863,Z_COORD])



	### PROCESS FILE ###
	for line in file:
		tokens = line.split(" ")
		if len(tokens) < 4:
			return
		#print "TOKENS: "+tokens[0]+" "+tokens[1]+" "+tokens[2]+" "+tokens[3]+"\n"
		print "Processing: "+tokens[0]+" "+tokens[1]+"\n"
		pos1 = [float(tokens[0]),float(tokens[1]), Z_COORD]

		

		dist = ((pos1[0]-current_pos[0]) ** 2 + (pos1[1]-current_pos[1]) ** 2) ** 0.5
		print "Distance: "+str(dist)+"Is in position: "+str(inPosition)
		if dist>0.1 and inPosition == True:
			print "Distance: "+str(dist)
			print "Raising pen..."
			current_joints = arm.joint_angle("right_s1")

			print "Current shoulder joint angle: "+str(current_joints)
			joint_command = {"right_s1": current_joints -0.1}
			arm.move_to_joint_positions(joint_command)

			print "Current shoulder joint angle: "+str(arm.joint_angle("right_s1"))
			#adjustZ(0.015)

		#print "Distance: "+str(dist)


		pos2 = [float(tokens[2]),float(tokens[3]), Z_COORD]
		recursive_drawing(current_pos,pos1,1)
		print "Processing: "+tokens[2]+" "+tokens[3]+"\n"
		recursive_drawing(current_pos,pos2,1)
		#getDistance()

	file.close()


if __name__ == "__main__":
    main()
