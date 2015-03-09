#!/usr/bin/env python

from operator import itemgetter
#Imposes a range of 0.35 <-> 0.75 on the x-axis and -0.4 <-> 0.4 on the y-axis, to conform to Baxter's strange coordinate system.
newtok = [0,0,0,0]
current_pos = [0,0]
points = []
distances = []

file1 = open("lines2.txt")
file2 = open('scaledLinesTEST.txt','w')
file3 = open("lines2TEST.txt",'w')
for line in file1:

	tokens = line.split(",")
	if len(tokens) < 4:
		continue
	#print tokens
	tokens[3] = tokens[3][:len(tokens[3])-1]
	xRange = float(5.0/6.0) * 0.45
	xOffset = (0.8 - xRange)/2
	file3.write("("+tokens[0]+","+tokens[1]+")\n("+tokens[2]+","+tokens[3]+")\n")

	newtok[0] = (xRange*float(tokens[0]))/500 - xOffset# - 0.4
	newtok[1] = (0.4*float(tokens[1]))/600 + 0.35
	newtok[2] = (xRange*float(tokens[2]))/500 - xOffset# - 0.4
	newtok[3] = (0.4*float(tokens[3]))/600 + 0.35

	### Preliminary reduction ###
	dist = ((newtok[2]-newtok[0]) ** 2 + (newtok[3]-newtok[1]) ** 2) ** 0.5
	distances.append(dist)
	if(dist < 0.01):

		points.append([round(newtok[0], 5),round(newtok[1], 5),"u"])
		# print points[len(points)-1]
		# file2.write("("+str(newtok[0])+","+str(newtok[1])+")\n")
	else:
		points.append([round(newtok[0], 5),round(newtok[1], 5),"u"])
		points.append([round(newtok[2], 5),round(newtok[3], 5),"u"])
		# file2.write("("+str(newtok[0])+","+str(newtok[1])+")\n("+str(newtok[2])+","+str(newtok[3])+")\n")
	current_pos[0] = newtok[2]
	current_pos[1] = newtok[3]



### REDUCE NUMBER OF POINTS ###
pcopy = list(points)
pcopy.sort(key=itemgetter(0))

print len(points)
d = 0
for i in xrange(0,len(pcopy)-1):
	for j in xrange(i+1,len(pcopy)):
		# print i,j,pcopy[i], pcopy[j]
		if pcopy[j][2] == "u":
			dist = float(((pcopy[j][0]-pcopy[i][0]) ** 2 + (pcopy[j][1]-pcopy[i][1]) ** 2) ** 0.5)

			if dist < 0.0035:
				d+=1
				points.remove(pcopy[j])
				pcopy[j][2] = "d"
print "Deleted: "
print d
print len(points)
# print points
for p in points:
	file2.write(str(p[0])+","+str(p[1])+"\n")


file1.close()
file2.close()
file3.close()
