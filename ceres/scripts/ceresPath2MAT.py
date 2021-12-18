#!/usr/bin/env python
# license
##############################################################################################
#  Script:        ceresPath2MAT
#  Version:       1.0
#  Authors:       Adrien Legrand
#  Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
#  Proyecto:      Trabajo de Grado
#  Goal:          Read a CERES Path from a .path file and generate a MatLab .Mat data file 
#		   for simulation.
#  Date:          08-12-2018
###############################################################################################
# Importations:
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from math import sqrt, cos, sin, pi
from ceres.msg import CeresRC
import tf
import utm
import os.path
import sys
###############################################################################################
# Functions Definitions:
# callbackRC: Acquire RC data, to check RC Emergency Stop state.
###############################################################################################
# Main Program
global gpsSub, initGPS, initOrientation, initPose, imuSub, AU
AU=True

# Check that the PathFile was added as argument when the Python Script was launched
if len(sys.argv)<2:
	raise IOError("Error: The algorithm takes exactly 1 argument!\nceresPathReader.py source.path")
# Check that the PathFile specified exists.
elif not os.path.isfile(sys.argv[1]):
	raise IOError("Error: Specified file doesn't exists!")
# Open the PathFile
else:
	file=open(sys.argv[1], "r")

###############################################################################################
# Decrypt the PathFile
l=file.readline()

# Initialize parameters
origins=[-1.0,-1.0,-1.0] # Origin: Lat, Long, Psi
originsFlag=[False,False,False]
frequency=-1

# Variable that will contains all points data.
points=[] # [[X, Y, Psi, U, W], [...]]

# Decrypt .path file
while(l!=""):
	# Split each line in space-separated elements
	elements=l.replace("\n", "").replace("\r","").split(" ")
	# Decode first keyword:
	if elements[0]=="POSITION":
		# Check if Origin Position is eather relative or GPS-coordinates defined.
		if elements[1]!="RELATIVE":
			raise NameError('This algorithm only works with relative paths!')

	elif elements[0]=="ORIENTATION":
		# Check if Orientation is eather relative or defined as an angle from East.
		if elements[1]!="RELATIVE":
			raise NameError('This algorithm only works with relative paths!')
	
	elif elements[0]=="FREQUENCY":
		# Read Path frequency (i.e. The time between each point)
		frequency=float(elements[1])

	elif elements[0][0]=="X":
		# if the first keyword is X, read the point data defined in the line.
		points.append([float(elements[0][1:]), float(elements[1][1:]), float(elements[2][3:])/180.*pi, float(elements[3][1:]), float(elements[4][1:])])	

	l=file.readline()


# Check that frequency has been speficied.
if(frequency<=0):
	raise ValueError("Error: Frenquency wasn't defined!")

outfile = open(sys.argv[1][:-4]+"m", "w")
t = "["
for i in range(len(points)):
	t+= str(i/frequency)
	if i<len(points)-1:
		t+=";"
t+="]"

outfile.write("Xd = timeseries([")
for i in range(len(points)):
	outfile.write(str(points[i][0]))
	if i<len(points)-1:
		outfile.write(";")
outfile.write("], ")
outfile.write(t)
outfile.write(");\n")

outfile.write("Yd = timeseries([")
for i in range(len(points)):
	outfile.write(str(points[i][1]))
	if i<len(points)-1:
		outfile.write(";")
outfile.write("], ")
outfile.write(t)
outfile.write(");\n")

outfile.write("Psid = timeseries([")
for i in range(len(points)):
	if i>1 and points[i][2]-points[i-1][2]>pi:
		points[i][2] = points[i][2]-2.*pi
	elif i>1 and points[i][2]-points[i-1][2]<-pi:
		points[i][2] = points[i][2]+2.*pi
	outfile.write(str(points[i][2]))
	if i<len(points)-1:
		outfile.write(";")
outfile.write("], ")
outfile.write(t)
outfile.write(");\n")

outfile.write("Ud = timeseries([")
for i in range(len(points)):
	outfile.write(str(points[i][3]))
	if i<len(points)-1:
		outfile.write(";")
outfile.write("], ")
outfile.write(t)
outfile.write(");\n")

outfile.write("Wd = timeseries([")
for i in range(len(points)):
	outfile.write(str(points[i][4]))
	if i<len(points)-1:
		outfile.write(";")
outfile.write("], ")
outfile.write(t)
outfile.write(");\n")
outfile.close()

