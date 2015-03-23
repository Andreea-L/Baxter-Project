#!/usr/bin/env python

import sys
import cv2
#sys.path = ['/usr/local/lib/python2.7/site-packages/'] + sys.path
#sys.path.remove('/opt/ros/hydro/lib/python2.7/dist-packages')
#sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages')
#import cv2 as cv3
#/hack
import os
import argparse
import rospy
import roslib
import baxter_interface
import numpy
import subprocess
import IKfinal
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import camera, head, CHECK_VERSION, Gripper

#cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
#change encoding to mono16 for greyscale or if you have compatibility issues

class ros2cv:

	global detector, scaling, cloth
	detector = 'finalDetector.py'
	scaling = "scaleFinal.py"
	cloth = 0

	def __init__(self):
		#set up cvbridge, subscribe to camera
		self.bridge = CvBridge()
		self.cap = cv2.VideoCapture(3)
		self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image,self.callback)
		#self.headcam = camera.CameraController('head_camera')
		#self.left = camera.CameraController('left_hand_camera')
		#self.right = camera.CameraController('right_hand_camera')
		#self.left.close()
		#self.right.open()
		#self.right.open()
		#self.right.resolution = (960, 600)
		#self.right.gain = -1
		#self.right.exposure = 90
		

	def callback(self,data):
		#rospy.sleep(1)
		self.face = head.Head()
		#self.face.set_pan(-1)
		cascade = cv2.CascadeClassifier("/home/level3_team/catkin_ws/src/level3_baxter/scripts/haarcascade_frontalface_alt.xml")
		nested = cv2.CascadeClassifier("/home/level3_team/catkin_ws/src/level3_baxter/scripts/haarcascade_eye.xml")
		try:
			#cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			ret, cv_image = self.cap.read()
			self.image_pub = rospy.Publisher('/robot/xdisplay',Image,latch=False)
			#cv2.imwrite("head photo.jpg", cv_image)
			#sys.stdout.write(".")
			#sys.stdout.flush()
			#rospy.sleep(2)
			#face.command_nod()
			#rospy.sleep(1)
			gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			gray = cv2.equalizeHist(gray)
			rects = detect(gray, cascade)
			vis = cv_image.copy()
			
			#cv2.imwrite('colour.jpg',vis)
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
			#cv2.imshow('facedetect', cv_image)
			#if cv2.waitKey(5) >0:
			#	self.image_sub = 0 ##THIS is how you stop callbacks. Find ascii code
			##for 'n' and then use that to stop this function once you are happy
		except CvBridgeError, e:
			print e

def main(args):
	global detector, y, yh, x,xw, scaling
	ic = ros2cv()
	rospy.init_node('ros2cv', anonymous = True)
	print "Make sure you are running me from level3_baxter/scripts"
	raw_input("Press Enter to take a snapshot...")
	ch = 'y\n'
	##interaction
	while True: #ch != 'n\n':
		print "Close pop-up window before answering..."
		print "Do you want to take another one, or change the detector?"
		print "Tablecloth detector: " + str(cloth)
		print "y/n/change"
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
			global cloth
			cloth = -cloth + 1
			
	print "Scaling points..."
	subprocess.call(['python '+scaling], shell=True)
	
	print "Starting drawing routine..."
	IKfinal.main()

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv2.CASCADE_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

if __name__ == '__main__':
	main(sys.argv)
	sys.exit()
