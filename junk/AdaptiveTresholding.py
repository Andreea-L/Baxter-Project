# import necessary packages

import numpy as np
import argparse
import cv2

# construct the argument parser and parse the arguments
#ap = argparse.ArgumentParser();
#ap.add_argument("-q", "--query", required= True, help = "Path to the query image")
#args = vars(ap.parse_args())

#load the query image, compute the ratio of the old height
#to the new height, clone it, and resize it
image = cv2.imread("face.jpg")#args["query"])
ratio = image.shape[0]/300.0

r = 500.0 / image.shape[1]
dim = (500, int(image.shape[0] * r))
 
# perform the actual resizing of the image and show it
resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
orig = image.copy()

denoised = cv2.fastNlMeansDenoisingColored(resized, None,10,10,7,21)

gray = cv2.cvtColor(denoised, cv2.COLOR_BGR2GRAY)

grayBlur = cv2.GaussianBlur(gray, (15,15),0)
threshGaussian = cv2.adaptiveThreshold(grayBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,1)
ret, threshGaussianOtsuThresh = cv2.threshold(grayBlur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)


medianBlur = cv2.medianBlur(gray, 5)
threshMedian = cv2.adaptiveThreshold(medianBlur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)



#contours
(cnts, _) = cv2.findContours(threshMedian.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
mask = np.zeros(resized.shape, np.uint8)
cv2.drawContours(mask,cnts, -1, (0, 255, 0), 1)

#canny
medianCanny = cv2.medianBlur(threshMedian, 5)
gaussianCanny=cv2.GaussianBlur(threshMedian, (15,15),0)
edged = cv2.Canny(gaussianCanny, 0, 200)

cv2.imshow("imageMedianBlurAdaptiveThresh", threshMedian)
cv2.imshow("imageGaussianBlurAdaptiveThresh", threshGaussian)
cv2.imshow("imageGaussianBlurOtsuThresh", threshGaussianOtsuThresh)
cv2.imshow("CannyMedianthresh", edged)
cv2.waitKey(0)
