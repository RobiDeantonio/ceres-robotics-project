#!/usr/bin/env python
# license

##############################################################################################
#  Script:        ceresPathGenerator
#  Version:       4.0
#  Authors:       Adrien Legrand
#  Organization:  Universidad Nacional de Colombia, Universidad Militar Nueva Grenada
#  Proyecto:      Trabajo de Grado
#  Goal:          Generate Speed Command to Control the CERES Agrobot and publish them to /ceres/cmd_vel topic
#  Date:          06-09-2018
###############################################################################################
# Importations:
import rospy
import datetime
import time
from geometry_msgs.msg import Twist
from ceres.msg import CeresRC
from math import sqrt,cos,sin
import sys

###############################################################################################
# Functions Definitions:
# adquire: Initialize ROS Node and all topics subscribers/publishers.
def adquire():
	rospy.init_node('PathPlanner', anonymous=True)
	rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
###############################################################################################
# callbackRC: receive RC Emergency Stop State to initiate/stop the sequence.
def callbackRC(data):
	global AU
	if data.emergency<50:
		if AU==True:
			rospy.logwarn("[PathGenerator]Emergency Stop Desactivated !")
		AU=False
	else:
		if AU==False:
			rospy.logwarn("[PathGenerator]Emergency Stop Activated !")
		AU=True
###############################################################################################
# generatePath: generate a path according to the selection
def generatePath(mode, length, Ax, Vx):
	global timeInit,pub

	rate=rospy.Rate(40)	# Frequency of the Reference Publication
	flag=True
	
	if mode==0:
		# Linear Path: Calculate Lenght
		Dx = length
	elif mode==1:
		# Circular Path: Calculate Lenght (Perimeter)
		Dx = length*3.14
	else:
		Dx = 0
	
	Ta = Vx / Ax # Acceleration Time
	Tx = Dx / Vx + Ta # Total Time
	
	while (not rospy.is_shutdown() and flag and not AU):
		# Initialize Message
		msg=Twist()

		# Generate a trapezoidal speed (Acceleration Phase, Constant Speed Phase, Decceleration Phase).
		
		if 2.0*Ta > Tx: # Case Maximal Velocity can't be reached (No Constant Speed Phase)
			Tx = 2*sqrt(Dx / Ax)
			# Update Path Total Time
			if time.time()-timeInit > Tx:
				# If the path is finished, stop the motors.
				flag = False
				msg.linear.x = 0.0
			
			elif time.time()-timeInit < Tx/2:
				# If Acceleration Phase, integrate the acceleration and publish the calculated speed.
				msg.linear.x = Ax * (time.time()-timeInit)
					
			else:
				# If Desacceleration Phase, integrate the acceleration and publish the calculated speed.
				msg.linear.x = Ax * (Tx/2) - Ax * (time.time() - timeInit - Tx/2)
					
					
		else: # Case Maximal Velocity is reached
			if time.time() - timeInit > Tx:
				# If the path is finished, stop the motors.
				flag = False
				msg.linear.x = 0.0
				
			elif time.time() - timeInit < Ta:
				# If Acceleration Phase, integrate the acceleration and publish the calculated speed.
					msg.linear.x=Ax*(time.time()-timeInit)
					
			elif time.time() - timeInit >= Ta and time.time() - timeInit < Tx - Ta:
				# If Constant Speed Phase, publish the Maximum speed.
				msg.linear.x = Ax * Ta
					
			else:
				# If Desacceleration Phase, integrate the acceleration and publish the calculated speed.
				msg.linear.x = Ax * Ta - Ax * (time.time() - timeInit - (Tx - Ta))

		if mode==1:
			# If Circular Path, calculate angular speed to respect the specified radius.
			msg.angular.z = 2.0*msg.linear.x / length
		else:
			# If Linear Path, angular speed is null.
			msg.angular.z = 0.0				

		# Publish the ROS message in the /ceres/cmd_vel topic.
		if not rospy.is_shutdown():
			pub.publish(msg)

		rate.sleep()


###############################################################################################
# Main Program
if __name__=='__main__':
	# Read the arguments that were entered when the script was executed.
	# If no argument: Linear 5 meters test.
	if len(sys.argv)==1:
		mode=0
		length = 5
	# 
	elif (len(sys.argv) != 3) or (not (unicode(sys.argv[1]).isnumeric())) or int(sys.argv[1])<0  or int(sys.argv[1])>2 or float(sys.argv[2])<0:
		raise Exception("Usage error: wrong parameters!")
	else:
		mode = int(sys.argv[1])
		length = float(sys.argv[2])

	#These Values can be adjusted:
	Vx = 0.5 #Maximal Speed
	Ax = 0.25 #Maximal Acceleration	

	#Do not adjust these maximum limits
	# Over-sized values Security Checks
	if(Ax>1):
		Ax=1
		rospy.logwarn("[PathGenerator]Linear Acceleration too high, setted to 1 [m.s-2]")

	if(Vx>1):
		Vx=1
		rospy.logwarn("[PathGenerator]Linear Speed too high, setted to 2 [m.s-1]")

	adquire()
	pub=rospy.Publisher('/ceres/cmd_vel',Twist,queue_size=25)
	AU=0

	if(AU==0):
		rospy.logwarn("[PathGenerator]Waitting Emergency Stop Activation...")
		while AU==0:
			pass
			
	rospy.logwarn("[PathGenerator]Waitting Emergency Stop Desactivation to initiate the test...")
	if mode==0:
		rospy.logwarn("[PathGenerator]THE ROBOT WILL PERFORM A "+str(length)+" METERS FORWARD MOVEMENT")
	elif mode==1:
		rospy.logwarn("[PathGenerator]THE ROBOT WILL PERFORM A CIRCULAR MOVEMENT WITH A DIAMETER OF "+str(length)+" METERS")

	
	while AU==1:
		pass

	rospy.loginfo("[PathGenerator]Running test...")
	timeInit=time.time()

	generatePath(mode, length, Ax, Vx)
	rospy.loginfo("[PathGenerator]Test is finished")

