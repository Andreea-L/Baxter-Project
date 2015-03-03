#!/usr/bin/env python

#Imposes a range of 0.35 <-> 0.75 on the x-axis and -0.2 <-> 0.2 on the y-axis, to conform to Baxter's strange coordinate system.
newtok = [0,0,0,0]

file1 = open("lines2.txt")
file2 = open('scaledLinesTEST.txt','w')
file3 = open("lines2TEST.txt",'w')
for line in file1:

	tokens = line.split(",")
	if len(tokens) < 4:
		print tokens
		continue
	#print tokens
	tokens[3] = tokens[3][:len(tokens[3])-1]
	xRange = float(5.0/6.0) * 0.45
	print xRange
	xOffset = (0.8 - xRange)/2
	file3.write("("+tokens[0]+","+tokens[1]+")\n("+tokens[2]+","+tokens[3]+")\n")

	newtok[0] = (xRange*float(tokens[0]))/500 - xOffset# - 0.4
	newtok[1] = (0.45*float(tokens[1]))/600 + 0.3
	newtok[2] = (xRange*float(tokens[2]))/500 - xOffset# - 0.4
	newtok[3] = (0.45*float(tokens[3]))/600 + 0.3
	file2.write(str(newtok[0])+","+str(newtok[1])+"\n"+str(newtok[2])+","+str(newtok[3])+"\n")

	

file1.close()
file2.close()
file3.close()
