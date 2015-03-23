#!/usr/bin/env python

f1 = open("/home/level3_team/catkin_ws/src/level3_baxter/scripts/lines2.txt")
f2 = open('linesScaled2.txt','w')
newtok = [0,0,0,0]

# token1 = 230
# token3 = 270
# newtok1 = 0.4*(1-(float(token1)/500)) -0.4*(float(token1)/500)
# newtok3 = 0.4*(1-(float(token3)/500)) -0.4*(float(token3)/500)

# token2 = 100
# newtok2 = 0.3*(1-(float(token2)/600)) +0.75*(float(token2)/600)

# print newtok1,newtok3
for line in f1:
	tokens = line.split(",")
	print tokens
	if float(tokens[0]) == 0 and float(tokens[1]) == 0 and float(tokens[2]) == 0 and float(tokens[3]) == 0:
		f2.write("FACE DONE\n")
	else:
		newtok[0] = 0.3*(1-(float(tokens[0])/600)) +0.75*(float(tokens[0])/600)
		newtok[2] = 0.3*(1-(float(tokens[2])/600)) +0.75*(float(tokens[2])/600)
		newtok[1] = 0.4*(1-(float(tokens[1])/500)) -0.4*(float(tokens[1])/500)
		newtok[3] = 0.4*(1-(float(tokens[3])/500)) -0.4*(float(tokens[3])/500)
		f2.write(str(newtok[0])+" "+str(newtok[1])+" "+str(newtok[2])+" "+str(newtok[3])+"\n")

