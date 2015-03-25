#!/usr/bin/env python

import sys
import cv2
import os
import argparse
import rospy
import roslib
import baxter_interface
import numpy
import subprocess
import InverseKinematics
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import camera, head, CHECK_VERSION, Gripper

########---------------------------------------------------------------------#########
######## This script is responsible for performing face detection			 #########
######## on the webcam video stream and integrating all of the				 #########
######## working parts of the pipeline (line detection, inverse kinematics). #########
######## This is the main runnable script. 									 #########
########---------------------------------------------------------------------#########
######## Authors: Robert McAlpine, Piotr Ozimek								 #########
########---------------------------------------------------------------------#########

class ros2cv:

	global detector, scaling, cloth
	detector = 'LineDetection.py'
	scaling = "Scale.py"
	cloth = 0	# Dark background flag

	def __init__(self):
		# Set up cvbridge, subscribe to camera
		self.bridge = CvBridge()
		self.cap = cv2.VideoCapture(3)
		self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image,self.callback)
		

	def callback(self,data):

		### FACE DETECTION ###
		self.face = head.Head()
		cascade = cv2.CascadeClassifier("/home/level3_team/catkin_ws/src/level3_baxter/scripts/haarcascade_frontalface_alt.xml")
		nested = cv2.CascadeClassifier("/home/level3_team/catkin_ws/src/level3_baxter/scripts/haarcascade_eye.xml")
		try:
			ret, cv_image = self.cap.read()
			self.image_pub = rospy.Publisher('/robot/xdisplay',Image,latch=False)

			gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			gray = cv2.equalizeHist(gray)
			rects = detect(gray, cascade)
			vis = cv_image.copy()
			
			for x1, y1, x2, y2 in rects:
				global y
				y = str(y1)
				global yh
				yh = str(y2)
				global x
				x = str(x1)
				global xw 
				xw= str(x2)
				roi = gray[y1:y2, x1:x2]
				bigGreyRoi = gray[y1-30:y2+30, x1-10:x2+10]
				vis_roi = vis[y1-30:y2+30, x1-10:x2+10]
				cv2.imwrite('face.jpg',bigGreyRoi)
				subrects = detect(vis_roi.copy(), nested)
				draw_rects(vis, rects, (0, 255, 0))
				#draw_rects(vis_roi, subrects, (255, 0, 0))
				cv2.imwrite('pic1.jpg', vis)
			screen_image = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
			self.image_pub.publish(screen_image)

		except CvBridgeError, e:
			print e

def main(args):
	global detector, y, yh, x,xw, scaling, cloth
	ic = ros2cv()
	rospy.init_node('ros2cv', anonymous = True)
	print "Make sure you are running me from level3_baxter/scripts"
	raw_input("Press Enter to take a snapshot...")
	ch = 'y\n'

	### USER INTERACTION ###
	while True: #ch != 'n\n':
		print "Close pop-up window before answering..."
		print "Using dark background detector?: " + str(cloth)
		print "Do you want to take another one, or change the detector?"
		print "y/n/change"

		# Call line detector on picture
		subprocess.call(['python '+detector+' '+y+' '+yh+' '+x+' '+xw+' '+str(cloth)], shell=True)
		ch = sys.stdin.readline()
		print "You entered "+ch
		if ch == "y\n" or ch == "Y\n":
			print "Taking another one..."
		if ch == "n\n" or ch == "N\n":
			print "Moving on..."
			break
		if ch == "change":
			print "Changing detector"
			cloth = -cloth + 1 		# Toggle dark background detector
			
	# Scale image
	print "Scaling points..."
	subprocess.call(['python '+scaling], shell=True)
	
	# Start drawing
	print "Starting drawing routine..."
	InverseKinematics.main()


# Performs OpenCV detection
def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv2.CASCADE_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

# Displays detection rectangle on camera stram
def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

if __name__ == '__main__':
	main(sys.argv)
	sys.exit()
