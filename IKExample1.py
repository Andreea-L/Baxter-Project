#!/usr/bin/env python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Example
"""
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


def ik_test(limb):
    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        0: PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.879328118102,
                    y= 0.429942645048,
                    z= -0.0214413443314,
                ),
                orientation=Quaternion(
                    x= -0.0469438028206,
                    y= 0.996503912113,
                    z=0.0217218637579,
                    w=0.0656078741904,
                ),
            ),
        ),
        1: PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= 0.322399701028,
                    y=0.490465361789,
                    z= -0.0214413443314,
                ),
                orientation=Quaternion(
                    x=-0.315535640909,
                    y= 0.948768632619,
                    z=-0.0129124068531,
                    w= 0.0104216516726,
                ),
            ),
        ),
        2: PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.799600883571,
                    y=-0.23749662701,
                    z= -0.0214413443314,
                ),
                orientation=Quaternion(
                    x= 0.43956774498,
                    y= 0.880545178035,
                    z=-0.0122520200635,
                    w=0.176834032406,
                ),
            ),
        ),
        3: PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.277785374042,
                    y=-0.429419583729,
                    z= -0.0214413443314,
                ),
                orientation= Quaternion(
                    x= 0.784746993853,
                    y= 0.595045943482,
                    z=-0.159715855262,
                    w=0.0677002685607,
                ),
            ),
        ),
     #    'right': PoseStamped(
     #        header=hdr,
     #        pose=Pose(
     #            position=Point(
     #                x=0.878059521973,
					# #x = 0.956982770038,
     #                y=-0.242579970338,
					# #y = -0.956982770038,
     #                z= 0.197567364084,
					# #z = 0,
     #            ),
     #            orientation=Quaternion(
     #                x= -0.00212093591552,
     #                y= 0.932525699193, #0.094253581186,
     #                z= -0.0357843940754,
     #                w=0.35931991194, #=0.260174958133,
     #            ),
     #        ),
     #    ),
    }
    while (i<3):
        ikreq.pose_stamp.append(poses[i])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
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
    	arm = baxter_interface.Limb(limb)
    	while not rospy.is_shutdown():
    		arm.set_joint_positions(limb_joints)
        	rospy.sleep(0.01)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

	
    return 0


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    return ik_test(args.limb)

if __name__ == '__main__':
    sys.exit(main())
