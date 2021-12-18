#!/usr/bin/env python
# license
##############################################################################################
#  Script:        ceresLogger
#  Version:       2.0
#  Authors:       Adrien Legrand
#  Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
#  Proyecto:      Trabajo de Grado
#  Goal:          Acquire and record pertinent data from the CERES Agrobot Platform. 
#  Date:          29-10-2018
###############################################################################################
# Importations:
import rospy
import datetime
import time
import tf
from ceres.msg import CeresArduinoLogging
from ceres.msg import CeresRC
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

###############################################################################################
# Functions Definitions:
# callbackRC: called each time a message is received from the RC Arduino.
def callbackRC(data):
	global var
	var[10][3]=data.CH1
	var[11][3]=data.CH2
	var[12][3]=data.emergency
	var[13][3]=data.AUX
###############################################################################################
# callbackPose: called each time a message is received from the Pose Controller.
def callbackPose(data):
	global var
	# Convert orientation from Quaternions to Euler Angles
	quaternion = (
	data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	var[22][3] = euler[2]
	
	var[20][3]=data.pose.pose.position.x
	var[21][3]=data.pose.pose.position.y
###############################################################################################
# callbackTwist: called each time a message is received from the cmd_vel topic.
def callbackTwist(data):
	global var
	var[8][3]=data.linear.x
	var[9][3]=data.angular.z
###############################################################################################
# callbackArduino: called each time a message is received from the Arduino.
def callbackArduino(data):
	global var
	var[6][3]=data.Ul
	var[7][3]=data.Ur
	var[18][3]=data.Vl
	var[19][3]=data.Vr
###############################################################################################
 #callBackIMU: called each time the IMU returns an Odometry message.
def callbackIMU(data):
	global var
	quaternion = (
	data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	var[3][3] = euler[2]
	
	var[1][3]=data.pose.pose.position.x
	var[2][3]=data.pose.pose.position.y

	var[4][3]=data.twist.twist.linear.x
	var[17][3]=data.twist.twist.linear.y
	var[5][3]=data.twist.twist.angular.z
###############################################################################################
 #callBackGPS: called each time the IMU returns a GPS Message.
def callbackGPS(data):
	global var
	var[14][3]=data.latitude
	var[15][3]=data.longitude
	var[16][3]=data.altitude
###############################################################################################
# adquire: Initialize ROS Node and all subscribers/publishers.
def adquire():
	rospy.init_node('CeresLogger', anonymous=True)
	rospy.Subscriber("/ceres/arduLog", CeresArduinoLogging, callbackArduino)
	rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
	rospy.Subscriber("/ceres/cmd_vel", Twist, callbackTwist)
	rospy.Subscriber("/advanced_navigation_driver/odom", Odometry, callbackIMU)
	rospy.Subscriber("/ceres/cmd_pose", Odometry, callbackPose)
	rospy.Subscriber("/advanced_navigation_driver/nav_sat_fix", NavSatFix, callbackGPS)
###############################################################################################
# writeLine: Publish a data set into the .CSV exported file.
def writeLine():
	global var, timeInit
	var[0][3]=time.time()-timeInit
	# For each element in the variables list, publish it into the CSV file.
	for i in range(len(var)):
		file.write(str(var[i][3]).replace(".",","))
		if(i<len(var)-1):
			file.write(";")
		else:
			file.write("\n")
###############################################################################################
# Main Program

if __name__=='__main__':
	rate = 20 # Recording Frequency

	# Define the global variables list for ROS Subscribers.
	var=[]
	var.append(["time","Time", "[s]", 0, False])				#0
	var.append(["Easting", "Robot Position East Axis", "[m]", 0, False]) 	#1
	var.append(["Northing", "Robot Position North Axis", "[m]", 0, False])	#2
	var.append(["heading", "Robot Heading to North", "[rad]", 0, False])	#3
	var.append(["v", "Robot Forward Speed", "[m/s]", 0, False])		#4
	var.append(["w", "Robot Angular Speed", "[rad/s]",  0, False])		#5
	var.append(["vl", "Left Driver Input Voltage", "[V]", 0, False])	#6
	var.append(["vr", "Right Driver Input Voltage", "[V]", 0, False])	#7
	var.append(["vd", "Reference Linear Speed", "[m/s]", 0, False])		#8
	var.append(["wd", "Reference Angular Speed", "[rad/s]", 0, False])	#9
	var.append(["CH1", "RC Channel 1 Value", "[%]", 0, False])		#10
	var.append(["CH2", "RC Channel 2 Value", "[%]", 0, False])		#11
	var.append(["AU", "RC Emergency Channel Value", "[%]", 0, False])	#12
	var.append(["AUX", "RC AUX Channel 1 Value", "[%]", 0, False])		#13
	var.append(["lat", "Robot Latitude", "[rad]", 0, False])		#14
	var.append(["lon", "Robot Longitude", "[rad]", 0, False])		#15
	var.append(["alt", "Robot Altitude", "[m]", 0, False])			#16
	var.append(["vy", "Robot Lateral Velocity", "[m/s]", 0, False])		#17
	var.append(["ErrU", "U PID Output Error", "[m/s]", 0, False])		#18
	var.append(["ErrW", "W PID Output Error", "[m/s]", 0, False])		#19
	var.append(["Ref Easting", "Ref Robot Position East Axis", "[m]", 0, False]) 	#20
	var.append(["Ref Northing", "Ref Robot Position North Axis", "[m]", 0, False])	#21
	var.append(["Ref Heading", "Ref Robot Heading to North", "[rad]", 0, False])	#22

	# Initialize ROS Node and Topics:
	adquire()

	# Initialize Time
	timeInit=time.time()
	now=datetime.datetime.now()

	# Create File with the date as filename
	filename="adquisition/"+str(now.day)+"-"+str(now.month)+"-"+str(now.year)+" "+str(now.hour)+"-"+str(now.minute)+"-"+str(now.second)+".csv"
	rospy.loginfo("[CSV-Log] Opening file "+filename+"...")
	file=open(filename,"w")
	time.sleep(0.1)
	rospy.loginfo("[CSV-Log] File Opened, recording...")

	# Write Values Header of Control Parameters
	file.write("KpU1,1;KiU0,65;KdU0;KpW0,8;KiW0,4;KdW0;KpP0.55;KiP0.5;KdP0.02\n")

	# Write Columns Titles/Headers into the CSV file.
	for j in range(3):
		for i in range(len(var)):
			file.write(var[i][j])
			if(i<len(var)-1):
				file.write(";")
			else:
				file.write("\n")
	
	# Record data at the specified Frequency.
	while(True):
		if((time.time()-timeInit)>var[0][3]+(1.0/rate)):
			writeLine()

