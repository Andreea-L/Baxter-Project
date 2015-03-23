#!/usr/bin/env python

import operator
import copy
from operator import itemgetter

# Imposes a range of 0.35 <-> 0.75 on the x-axis and -0.4 <-> 0.4 on the y-axis for all of the detected points,
# to conform to Baxter's strange coordinate system.
# Additionally, it removes points that are too close together, making sure the number of overlapping points is minimal.


### Find the point closes to "point" in the given array
def closest(point,array):
	closestPoint =[0,0]
	closestDist = 100
	closestIndex = 0
	for i in xrange(0,len(array)):
		dist = float(((array[i][0]-point[0]) ** 2 + (array[i][1]-point[1]) ** 2) ** 0.5)
		if dist < closestDist:
			closestDist = dist;
			closestPoint = [array[i][0],array[i][1]]
			closestIndex = i
	return closestPoint

def main():
	newtok = [0,0,0,0]
	points = []
	distances = []

	f = open("yobackFinalProcessed.txt",'r+')


	### PERFORM SCALING AND PRELIMINARY POINT REDUCTION ###
	for line in f:

		tokens = line.split(",")
		print tokens
		if len(tokens) < 4:
			break;
		#tokens[3] = tokens[3][:len(tokens[3])-1]

		### Scaling ###
		xRange = float(5.0/6.0) * 0.45
		xOffset = (0.8 - xRange)/2

		newtok[0] = (xRange*float(tokens[0]))/500 - xOffset# - 0.4
		newtok[1] = (0.4*float(tokens[1]))/600 + 0.35
		newtok[2] = (xRange*float(tokens[2]))/500 - xOffset# - 0.4
		newtok[3] = (0.4*float(tokens[3]))/600 + 0.35

		## Preliminary reduction ###
		# dist = ((newtok[2]-newtok[0]) ** 2 + (newtok[3]-newtok[1]) ** 2) ** 0.5
		# distances.append(dist)
		points.append([round(newtok[0], 5),round(newtok[1], 5)])
		#if(dist >= 0.01):
		points.append([round(newtok[2], 5),round(newtok[3], 5)])
			


	### FURTHER REDUCE NUMBER OF POINTS ###

	pcopy = copy.deepcopy(points)
	pcopy.sort(key=itemgetter(0))

	for i in xrange(0,len(pcopy)):
		pcopy[i].append("u")


	d = 0
	for i in xrange(0,len(pcopy)-1):
		for j in xrange(i+1,len(pcopy)):
			if pcopy[j][2] == "u":
				dist = float(((pcopy[j][0]-pcopy[i][0]) ** 2 + (pcopy[j][1]-pcopy[i][1]) ** 2) ** 0.5)

				if dist < 0.002:
					d+=1
					points.remove([pcopy[j][0],pcopy[j][1]])
					pcopy[j][2] = "d"

	print "Deleted: "+str(d) +"points from file."
	print "Resulting number of points: "+str(len(points))




	### ORDER BT CLOSEST POINTS, FOR MORE ACCURATE DRAWING ###
	points.sort(key=operator.itemgetter(0))
	sortcopy = list(points)


	cPoints =  []
	next = sortcopy[0]

	while(len(points)!=0):
		cPoint = closest(next,points)
		cPoints.append(cPoint)
		points.remove(cPoint)

		next = cPoints[-1]

	#print points
	print "DONE"
	f.seek(0)
	f.truncate()
	### WRITE OUT POINTS ###
	for i in xrange(0,len(cPoints)):
		if cPoints[i][0]>-0.17 and cPoints[i][0]<0.12:
			print "Cropping..."
			f.write(str(cPoints[i][0])+" "+str(cPoints[i][1])+" ")
			if i%2==1 :
				f.write("\n")


	f.close()

if __name__ == "__main__":
    main()
