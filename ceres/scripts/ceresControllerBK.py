#!/usr/bin/env python
# license
##############################################################################################
#  Script:        ceresController
#  Version:       2.0
#  Authors:       Adrien Legrand
#  Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
#  Proyecto:      Trabajo de Grado
#  Goal:          Run the Position Control for the CERES Agrobot.
#  Date:          29-10-2018
###############################################################################################
# IMPORTATIONS
import rospy
from math import sin, cos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import time

###############################################################################################
# calculateSpeeds: Calculate the position errors and generate/publish the output in the /ceres/cmd_vel topic.
def calculateSpeeds():
	global var, Kp, Ki, Kd
	#Calculate position error in the inertial referencial.
	Xe = var[3] - var[0]
	Ye = var[4] - var[1]
	
	#Calculate position error in the robot ENU referencial.
	Xer = cos(var[2]) * Xe + sin(var[2]) * Ye
	Yer = -sin(var[2]) * Xe + cos(var[2]) * Ye
	Psier = var[5] - var[2]

	#Verify that the angle is the minimim, otherwise uses the error in the other direction.
	if Psier > 3.14:
		Psier = Psier - 6.28
	elif Psier < -3.14:
		Psier = Psier + 6.28
	
	#Calculate the controller output.
	u = Kp * Xer + var[6] *cos(Psier)
	w = var[7] + Kd * var[6] * Yer + Ki * sin(Psier)
	
	#Publish output to the MainArduino via the /ceres/cmd_vel topic
	msg=Twist()
	msg.linear.x = u
	msg.angular.z = w
	pub.publish(msg)
###############################################################################################
#callBackIMU: called each time the IMU return an Odometry message.
def callbackIMU(data):
	global var, lastIMUTime
	quaternion = (
	data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	var[2] = euler[2] 
	
	var[0]=data.pose.pose.position.x
	var[1]=data.pose.pose.position.y

	now = rospy.get_rostime()
	time = now.secs+now.nsecs*pow(10,-9)
	lastIMUTime = time
###############################################################################################	
 #callBackRef: called each time a Reference Odometry message is received.
def callbackRef(data):
	global var, lastRefTime
	quaternion = (
	data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	var[5] = euler[2] 
	
	var[3]=data.pose.pose.position.x
	var[4]=data.pose.pose.position.y

	var[6]=data.twist.twist.linear.x
	var[7]=data.twist.twist.angular.z

	now = rospy.get_rostime()
	time = now.secs+now.nsecs*pow(10,-9)
	lastRefTime = time
###############################################################################################
# adquire: Initialize all the subscribers/publishers.
def adquire():
	global pub
	rospy.init_node('CeresController', anonymous=True)
	rospy.Subscriber("/ceres/cmd_pose", Odometry, callbackRef)
	rospy.Subscriber("/advanced_navigation_driver/odom", Odometry, callbackIMU)
	pub=rospy.Publisher('/ceres/cmd_vel',Twist,queue_size=25)
###############################################################################################
# MAIN PROGRAM
if __name__ == "__main__":
	global var, Kp, Ki, Kd, pub, lastIMUTime, lastRefTime
	# Controller Parameters:
	Kp = 0.7	# X_robot
	Kd = 0.08	# Y_robot
	Ki = 0.22	# Psi_robot

	#Initialize Global Variables for ROS Callbacks
	var=[]
	var.append(0.0) # Ximu
	var.append(0.0) # Yimu
	var.append(0.0) # Psiimu
	var.append(0.0) # Xref
	var.append(0.0) # Yref
	var.append(0.0) # Psiref
	var.append(0.0) # Uref
	var.append(0.0) # Wref
	
	#Initialize ROS Node and topics
	adquire()
	time.sleep(0.5)

	#Initilize time variables for Timeout verifications
	now = rospy.get_rostime()
	time = now.secs+now.nsecs*pow(10,-9)
	lastIMUTime = time
	lastRefTime = time
	IMUFlag = False #Flag to avoid notification repetition when IMU Timeout
	RefFlag = False #Flag to avoid notification repetition when Ref Timeout

	rate=rospy.Rate(20) #Controller Frequency [Hz]

	# Main Loop:	
	while (not rospy.is_shutdown()):
		now = rospy.get_rostime()
		time = now.secs+now.nsecs*pow(10,-9)
		# Check last IMU Received Message Date
		if time > lastIMUTime+0.5:
			# Notify IMU Timeout
			if not IMUFlag:
				rospy.logwarn("[Pose Controller] IMU Timeout")
				IMUFlag = True
		# Check last Ref Received Message Date		
		elif time > lastRefTime+0.5:
			# Notify Ref Timeout
			if not RefFlag:
				rospy.logwarn("[Pose Controller] Reference Timeout")
				RefFlag = True
		# If all messages are up to date, apply control.		
		else:
			calculateSpeeds()
			RefFlag = False
			IMUFlag = False
		rate.sleep()
