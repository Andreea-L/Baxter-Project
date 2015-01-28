import argparse
import struct
import sys
import baxter_interface

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

### CONSTANTS ###
Z_COORD = -0.0230625941977
ROTATION = [-0.124120878028, 0.991890846532, 0.00478437869368,-0.0269010394494]

### GLOBALS ###
joints = []
current_pos = []
arm = None
ns, iksvc, ikreq, hdr

def recursive_drawing(start_position, end_position,rec_depth):
	global current_pos

	if rec_depth > 10:
		print "Recursing limit reached."
		return 0

	pose = {
	    0: PoseStamped(
	        header=hdr,
	        pose=Pose(
	            position=Point(
	                x= end_position[0],
	                y= end_position[1],
	                z= end_position[2],
	            ),
	            orientation=Quaternion(
	                x= ROTATION[0],
	                y= ROTATION[1],
	                z= ROTATION[2],
	                w= ROTATION[3],
	            ),
	        ),
	    )}

	ikreq.pose_stamp.append(pose)
	
	try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
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
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
		# reformat the solution arrays into a dictionary
    	#joint_solution = dict(zip(resp.joints[0].names, resp.joints[0].angles))
    	# set arm joint positions to solution
    	
    	arm.set_joint_positions(limb_joints)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        mid_position = [(start_position[0]+end_position[0])/2,(start_position[1]+end_position[1])/2,Z_COORD]
		print "Splitting at: "+str(mid_position)
		recursive_drawing(start_position,mid_position,rec_depth+1)
		recursive_drawing(mid_position,end_position,rec_depth+1)
		#rospy.sleep(0.1)
	current_pos = end_position	


def main():
	global ns, iksv, ikreq,hdr, current_pos, arm

	rospy.init_node("rsdk_ik_service_client")
	ns = "ExternalTools/left/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	arm = baxter_interface.Limb('left')

	file = open("/home/andreea/BaxterProject/scaledLinesDetected2.txt")
	current = arm.endpoint_pose()
	current_pos = [current['position'].x,current['position'].y,current['position'].z]
	for line in file:
		tokens = line.split(" ")
		print "Processing: "+tokens[0]+" "+tokens[1]+"\n"
		pos1 = [float(tokens[0]),float(tokens[1]), Z_COORD]
		pos2 = [float(tokens[2]),float(tokens[3]), Z_COORD]
		recursive_drawing(current_pos,pos1,1)
		print "Processing: "+tokens[1]+" "+tokens[3]+"\n"
		recursive_drawing(current_pos,pos2,1)

	file.close()
	pose = 

	

	
    return 0


if __name__ == "__main__":
    main()