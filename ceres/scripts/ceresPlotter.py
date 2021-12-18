#!/usr/bin/env python
# license

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D
from ceres.msg import CeresOdometry
from math import ceil

xRef=[]
yRef=[]

xReal=[]
yReal=[]

def plot(x,y,source):
	global xReal,yReal,xRef,yRef,lineReal,lineRef,ax

	if source==0:
		xRef.append(x)
		yRef.append(y)
	else:
		xReal.append(x)
		yReal.append(y)

	m=int(ceil(max([max(xReal+[0]),max(xRef+[0]),max(yReal+[0]),max(yRef+[0])])/10.0))*10

	lineRef.set_xdata(xRef)
	lineRef.set_ydata(yRef)

	lineReal.set_xdata(xReal)
	lineReal.set_ydata(yReal)

	ax.axis("equal")
	ax.axis([-m,m,-m,m])

	plt.title("Robot Position in [X,Y] Referential")
	plt.xlabel("X Position [m]")
	plt.ylabel("Y Position [m]")

	plt.grid(True)

	plt.draw()
	plt.pause(10**(-99))

def callback(data):
    plot(data.x,data.y,0)

def callbackIMU(data):
    plot(data.x,data.y,1)

def listener():
	rospy.init_node('ceresPlotter')
	rospy.Subscriber("/ceres/Odometry2", CeresOdometry, callbackIMU)
	rospy.Subscriber("/ceres/refPos", Pose2D, callback)

	#Agregar Listener IMU

if __name__=='__main__':
	plt.ion()
	ax=plt.gca()
	lineRef,=ax.plot(xRef,yRef,label='Reference',color='blue')
	lineReal,=ax.plot(xReal,yReal,label='Measured',color='red')
	plt.legend([lineRef,lineReal],['Reference Position','Measured Position'])

	listener()
	plt.show(block=True)
