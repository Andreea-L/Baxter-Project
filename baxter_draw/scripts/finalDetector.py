# import necessary packages

import numpy as np
import argparse
import sys
import getopt

sys.path = ['/usr/local/lib/python2.7/site-packages/'] + sys.path
#optional commands, to doubly make sure that opencv2 dir is loaded last.
sys.path.remove('/opt/ros/hydro/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages') 

import cv2 #this is now opencv3

#function to check whether a point was already written in the file as part of face line detection
def checkPointOutsideFace(x1,y1,x,xw,y,yh):
		
		if(x1>=x) and (x1<=xw) and (y1>=y) and (y1<=yh):
				return False
		return True


def main(argv):	

	#read image
	faceName = "face.jpg"
	originalImage = cv2.imread(faceName)

	#open the file for outputing coordinates
	fileName = 'linesDetected.txt'
	f = open(fileName,'w')

	# perform resizing of the image
	resized = cv2.resize(originalImage,(500,600))

	#coordinates from face detection should go in here
	y = int(argv[0])*600/480
	yh = int(argv[1])*600/480
	x = int(argv[2])*500/640
	xw = int(argv[3])*500/640

	#crop face
	crop = resized[y:yh,x:xw]

	#remove noise
	denoisedFace = cv2.fastNlMeansDenoisingColored(crop, None,10,10,7,21)
	denoisedExtern= cv2.fastNlMeansDenoisingColored(resized, None,10,10,7,21)

	#grayscale
	grayFace = cv2.cvtColor(denoisedFace, cv2.COLOR_BGR2GRAY)
	grayExtern = cv2.cvtColor(denoisedExtern, cv2.COLOR_BGR2GRAY)

	#change contrast
	grayContrastedFace = cv2.equalizeHist(grayFace)
	grayContrastedExtern = cv2.equalizeHist(grayExtern)

	##################PROCESSING OF ONLY THE FACE#######################
	tablecloth = int(argv[4])

	if tablecloth:
		medianBlurFace = cv2.medianBlur(grayFace, 5)
		threshFace = cv2.adaptiveThreshold(medianBlurFace, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,7,2)
	else:
		gaussianBlurFace = cv2.GaussianBlur(grayContrastedFace, (9,7),0)
		threshFace = cv2.adaptiveThreshold(gaussianBlurFace, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,15,4)

	#create face image
	width = crop.shape[1]
	height = crop.shape[0]
	faceImage = np.zeros((height, width, 3), np.uint8)

	#cv2.imshow("face", threshGaussianFace)

	#create line segment detector
	if tablecloth:
		detectorFace = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD ,0.97, 0.6, 0.8, 40, 0, 0.90, 1024)
	else:
		detectorFace = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD ,0.97, 0.6, 0.8, 60, 0, 0.90, 1024)
	
	#detect lines
	lines = detectorFace.detect(threshFace)
	for line in lines:
		detectorFace.drawSegments(faceImage,line)


	#write the detected lines to the file
	for line in lines[0]:
		x1 = line[0][0]
		y1 = line[0][1]
		x2 = line[0][2]
		y2 = line[0][3]
		f.write(str(x1+x)+','+str(y1+y)+','+str(x2+x)+','+str(y2+y)+'\n')	
			
	f.write('0,0,0,0\n')
	##################PROCESSING OF HAIR#######################
	#External blur for hair

	if tablecloth:
		gaussianBlurExtern = cv2.GaussianBlur(grayContrastedExtern, (11,11),0)
		threshExtern = cv2.adaptiveThreshold(gaussianBlurExtern, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,19,3)
	else:
		gaussianBlurExtern = cv2.GaussianBlur(grayContrastedExtern, (11,11),0)
		threshExtern = cv2.adaptiveThreshold(gaussianBlurExtern, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,19,4)

	#cv2.imshow("Gaussian", threshGaussianExtern)

	#create new big external image - for hair
	width = resized.shape[1]
	height = resized.shape[0]
	externImage = np.zeros((height, width, 3), np.uint8)

	#create detector
	if tablecloth:
		detectorExtern = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD ,0.97, 0.8, 0.8, 60, 0, 0.55, 1024)
	else:
		detectorExtern = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD ,0.97, 0.8, 0.8, 60, 0, 0.7, 1024)

	#detect lines
	lines = detectorExtern.detect(threshExtern)
	for line in lines:
		detectorExtern.drawSegments(externImage,line)
			
	#write the detected lines to the file
	for line in lines[0]:
		x1 = line[0][0]
		y1 = line[0][1]
		x2 = line[0][2]
		y2 = line[0][3]
		if(checkPointOutsideFace(x1,y1,x,xw,y,yh) and checkPointOutsideFace(x2,y2,x,xw,y,yh)):
			f.write(str(x1)+','+str(y1)+','+str(x2)+','+str(y2)+'\n')

	f.close()
	#put the face on top of the external image
	externImage[y:yh,x:xw]=faceImage
	cv2.imshow("",externImage)
	outputImage = faceName[:-4] + "ProcessedFinal.jpg"
	cv2.imwrite(outputImage, externImage)

	cv2.waitKey(0)

if __name__ == "__main__":
	main(sys.argv[1:])
