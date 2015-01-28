#!/usr/bin/env python

import rospy
import numpy
import sys

import baxter_interface
from baxter_pykdl import baxter_kinematics

### FILE PATH ###
PATH = "/home/andreea/BaxterProject/scaledLinesDetected2.txt"

### CONSTANTS ###
Z_COORD = -0.0230625941977
ROTATION = [-0.124120878028, 0.991890846532, 0.00478437869368,-0.0269010394494]

### GLOBALS ###
joints = []
current_pos = []
arm = None


### EXECUTE MOVEMENT AND RECURSIVELY SPLIT FAILING SEGMENTS ###
def recursive_drawing(start_position, end_position, rec_depth):
	global current_pos

	kin = baxter_kinematics('left')
	result = kin.inverse_kinematics(end_position,ROTATION)
	#print result
	if rec_depth > 10:
		print "Recursing limit reached."
		return 0
	if result == -1:
		mid_position = [(start_position[0]+end_position[0])/2,(start_position[1]+end_position[1])/2,Z_COORD]
		print "Splitting at: "+str(mid_position)
		recursive_drawing(start_position,mid_position,rec_depth+1)
		recursive_drawing(mid_position,end_position,rec_depth+1)
	else:
		print "Solution found"
		joint_positions = dict(zip(joints,result.tolist()))
		print result
		arm.move_to_joint_positions(joint_positions)
		#rospy.sleep(0.1)
		current_pos = end_position		


def main():

	### SETUP ###
	global joints, arm, current_pos
	rospy.init_node('baxter_kinematics')
	arm = baxter_interface.Limb('left')
	joints = arm.joint_names()

	print '*** Baxter PyKDL Kinematics ***\n'
	
	
	file = open(PATH)
	current = arm.endpoint_pose()
	current_pos = [current['position'].x,current['position'].y,current['position'].z]
	#print current_pos
	#recursive_drawing(current_pos,[0.066917404475,-0.343215954863,Z_COORD])


	### PROCESS FILE ###
	for line in file:
		tokens = line.split(" ")
		print "Processing: "+tokens[0]+" "+tokens[1]+"\n"
		pos1 = [float(tokens[0]),float(tokens[1]), Z_COORD]
		pos2 = [float(tokens[2]),float(tokens[3]), Z_COORD]
		recursive_drawing(current_pos,pos1,1)
		print "Processing: "+tokens[1]+" "+tokens[3]+"\n"
		recursive_drawing(current_pos,pos2,1)

	file.close()


if __name__ == "__main__":
    main()