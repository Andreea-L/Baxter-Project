# import necessary packages

import numpy as np
import argparse
import cv2


#load image and resize
im = cv2.imread('kidselfie.jpg')

r = 500.0 / im.shape[1]
dim = (500, int(im.shape[0] * r))
resized = cv2.resize(im, dim, interpolation = cv2.INTER_AREA)

#apply grayscale to image and modify contrast
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
gray = cv2.equalizeHist(gray)

#Gaussian blur and adaptive thresholding
grayBlur = cv2.GaussianBlur(gray, (7,9),0)
threshGaussian = cv2.adaptiveThreshold(grayBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,1)


#median blur and adaptive thresholding 
medianBlur = cv2.medianBlur(gray, 7)
threshMedian = cv2.adaptiveThreshold(medianBlur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)



#create new blank image
width = 500
height = int(im.shape[0] * r)
image = np.zeros((height, width, 3), np.uint8)


#line segment detection
detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD ,0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024)
lines = detector.detect(threshGaussian)
for line in lines:
    detector.drawSegments(image,line)

##optional 
#lines = detector.detect(threshMedian)
#for line in lines:
#    detector.drawSegments(image,line)
##optional


#write the detected lines to the file
f = open('linesDetected.txt','w')
for line in lines[0]:
    x1 = line[0][0]
    y1 = line[0][1]
    x2 = line[0][2]
    y2 = line[0][3]
    f.write(str(x1)+','+str(y1)+','+str(x2)+','+str(y2)+'\n')
   

f.close()

#display image
cv2.imshow("TEST", image)
#cv2.imshow("ThreshMedian", threshMedian)
#cv2.imshow("ThreshGaussian", threshGaussian)


cv2.waitKey(0)
