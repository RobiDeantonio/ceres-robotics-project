#!/usr/bin/env python
# license
##############################################################################################
#  Script:        ceresPathReader
#  Version:       2.0
#  Authors:       Adrien Legrand
#  Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
#  Proyecto:      Trabajo de Grado
#  Goal:          Read a CERES Path from a .path file and publish ROS Pose messages.
#  Date:          28-10-2018
###############################################################################################
# Importations:
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from math import sqrt, cos, sin
from ceres.msg import CeresRC
import tf
import utm
import os.path
import sys
###############################################################################################
# Functions Definitions:
# callbackRC: Acquire RC data, to check RC Emergency Stop state.
def callbackRC(data):
	global AU
	if data.emergency<50:
		if AU==True:
			rospy.logwarn("Emergency Stop Desactivated !")
		AU=False
	else:
		if AU==False:
			rospy.logwarn("Emergency Stop Activated !")
		AU=True
###############################################################################################
# callbackGPS: Acquire Robot GPS Initial Position
def callbackGPS(msg):
	global initGPS
	initGPS=[msg.latitude, msg.longitude]
	gpsSub.unregister() #Once first message is received, disconnect the subscriber.
###############################################################################################
# callbackGPS: Acquire Robot Initial UTM Position/Orientation
def callbackImu(msg):
	global initOrientation, initPose, imuSub
	# Convert from Quaternions to Euler Angles
	quaternion = (
	msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y,
	msg.pose.pose.orientation.z,
	msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	
	initPose[0]=msg.pose.pose.position.x
	initPose[1]=msg.pose.pose.position.y
	initOrientation=euler[2]
	imuSub.unregister()	#Once first message is received, disconnect the subscriber.
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
			originsFlag[0]=True
			originsFlag[1]=True
			origins[0]=float(elements[1])
			origins[1]=float(elements[2])

	elif elements[0]=="ORIENTATION":
		# Check if Orientation is eather relative or defined as an angle from East.
		if elements[1]!="RELATIVE":
			originsFlag[2]=True
			origins[2]=float(elements[1])/180.*3.14
	
	elif elements[0]=="FREQUENCY":
		# Read Path frequency (i.e. The time between each point)
		frequency=float(elements[1])

	elif elements[0][0]=="X":
		# if the first keyword is X, read the point data defined in the line.
		points.append([float(elements[0][1:]), float(elements[1][1:]), float(elements[2][3:])/180.*3.14, float(elements[3][1:]), float(elements[4][1:])])	

	l=file.readline()


# Initialize Robot Initial position variables
initGPS=[-1000.0, -1000.0]
initOrientation=-1000.0
initPose=[0, 0]

# Initialize ROS Node and Topics 
rospy.init_node('ceresPathReader')
gpsSub = rospy.Subscriber('/advanced_navigation_driver/nav_sat_fix', NavSatFix, callbackGPS)
imuSub = rospy.Subscriber('/advanced_navigation_driver/odom', Odometry, callbackImu)
rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
pub=rospy.Publisher('/ceres/cmd_pose',Odometry,queue_size=25)

# Wait for receving IMU data.
rospy.logwarn("[Pose-Controller]Waiting IMU Initial Position...")
while(initPose[0]==0 and initPose[1]==0):
	pass
rospy.loginfo("[Pose-Controller]IMU Initial Position Received")
rospy.loginfo("[Pose-Controller]     - X: "+str(initPose[0]))
rospy.loginfo("[Pose-Controller]     - Y: "+str(initPose[1]))

rospy.loginfo("[Pose-Controller]Waiting IMU Initial Orientation...")
rospy.loginfo("[Pose-Controller]IMU Initial Orientation Received")
rospy.loginfo("[Pose-Controller]     - Orientation: "+str(initOrientation))

# Wait for receving GPS data.
if (originsFlag[0]==True or originsFlag[1]==True):
	rospy.logwarn("[Pose-Controller]Waiting GPS Initial Position...")
	while(initGPS[0]==-1000.0 and initGPS[1]==-1000.0):
		pass
	rospy.loginfo("[Pose-Controller]GPS Initial Position Received")
	rospy.loginfo("[Pose-Controller]     - Latitude: "+str(initGPS[0]))
	rospy.loginfo("[Pose-Controller]     - Longitude: "+str(initGPS[1]))

# Check that frequency has been speficied.
if(frequency<=0):
	raise ValueError("Error: Frenquency wasn't defined!")
timer=rospy.Rate(frequency)

# Initialize Angle Offset
if originsFlag[2]==True:
	# If Absolute Angle, Offset is specified from East Axis.
	dAngle = origins[2]
else:
	# If relative Angle, Offset is specified from Initial Robot X Axis.
	dAngle = initOrientation

# Initialize Position Offset
if (originsFlag[0]==True or originsFlag[1]==True):
	# If Absolute Position, convert Robot Initial GPS and Path Origin GPS Position to UTM (Easting, Northing) Positions
	robotInitPose = utm.from_latlon(initGPS[0], initGPS[1])
	csvInitPose = utm.from_latlon(origins[0], origins[1])

	# Calculate the distance around each axis (Easting, Northing) of the Path Origin from the Robot Initial Position.
	deltaX = csvInitPose[0]-robotInitPose[0] 
	deltaY = csvInitPose[1]-robotInitPose[1] 
	
	# Check if The Path Origin is around the robot initial position (Check the norm)
	distance = sqrt(pow(deltaX,2)+pow(deltaY,2))
	if(distance>10):
		# If The Path Origin is too far, raise error.
		raise ValueError("ERROR: Robot is too far from the CSV Origin Position ("+str(distance)+" m)")
	
	# Compute the Position offset, respectively to the IMU Origin (0,0)
	dX = deltaX + initPose[0]
	dY = deltaY + initPose[1]

else:
	# If Relative Position, Offset is the robot initial position, respectively to the IMU Origin (0,0)
	dX = initPose[0]
	dY = initPose[1]

# Wait for Emergency Stop Activation before initiating Path.
rospy.logwarn("[Pose-Controller]Waitting for Emergency Stop Activation...")
while AU!=True:
	pass

rospy.loginfo("[Pose-Controller]Sequence loaded... "+str(len(points))+" points detected.")

# Wait for Emergency Stop Desactivation to initiate the Path.
rospy.logwarn("[Pose-Controller]Waitting for Emergency Stop Desactivation to EXECUTE the sequence.")
while AU!=False:
	pass

# Publish the Path Points, at the specified Frequency.
for i in range(len(points)):
	pubMsg = Odometry()
	pubMsg.twist.twist.linear.x = points[i][3]
	pubMsg.twist.twist.angular.z = points[i][4]

	# Convert from the Path Referential to the Inertial Referential (UTM)
	if originsFlag[2]==False:
		pubMsg.pose.pose.position.x = points[i][0]*cos(dAngle) - points[i][1]*sin(dAngle) + dX
		pubMsg.pose.pose.position.y = points[i][1]*cos(dAngle) + points[i][0]*sin(dAngle) + dY
	else:
		pubMsg.pose.pose.position.x = points[i][0] + dX
		pubMsg.pose.pose.position.y = points[i][1] + dY

	quaternions = tf.transformations.quaternion_from_euler(0.,0.,points[i][2]+dAngle)
	pubMsg.pose.pose.orientation.x = quaternions[0]
	pubMsg.pose.pose.orientation.y = quaternions[1]
	pubMsg.pose.pose.orientation.z = quaternions[2]
	pubMsg.pose.pose.orientation.w = quaternions[3]

	# Publish the Odom Message to /ceres/cmd_pose topic
	pub.publish(pubMsg)

	timer.sleep()

rospy.loginfo("[Pose-Controller]Sequence Finished, Node shutting down.")
