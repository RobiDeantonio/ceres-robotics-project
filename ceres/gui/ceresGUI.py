#!/usr/bin/env python
# license
import sys
import os
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QObject, QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage, QTransform, QPixmap, QTextCursor
from PyQt5.QtWidgets import QFileDialog
from mainwindow import Ui_MainWindow
import subprocess32 as subprocess
import time
import tf
import rospy
import qOSM 
qOSM.use("PyQt5")
from qOSM.common import QOSM
sys.path.append("/root/catkin_ws/src")
from ceres.msg import CeresArduinoLogging
from ceres.msg import CeresRC
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Log
from math import cos, sin, sqrt
from serial.tools import list_ports
import pyqtgraph
import utm

class CeresApplication(QtWidgets.QMainWindow):
	def __init__(self, parent=None):
		super (CeresApplication, self).__init__(parent)
		self.createWidgets()
		self.path=""

	def createWidgets(self):
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
  
		self.ui.pushButton.clicked.connect(self.connectIMU)
		self.ui.pushButton_3.clicked.connect(self.connectMainArduino)
		self.ui.pushButton_2.clicked.connect(self.connectRCArduino)
		self.ui.pushButton_5.clicked.connect(self.startPathGenerator)
		self.ui.pushButton_6.clicked.connect(self.stopPathGenerator)
		self.ui.pushButton_7.clicked.connect(self.detectIMU)
		self.ui.pushButton_8.clicked.connect(self.detectArduinoRC)
		self.ui.pushButton_9.clicked.connect(self.detectMainArduino)
		self.ui.pushButton_4.clicked.connect(self.loadPath)
		
		self.ui.verticalSlider.valueChanged.connect(refreshTrajectory)
		self.ui.doubleSpinBox.valueChanged.connect(refreshTrajectory)
		self.ui.doubleSpinBox_2.valueChanged.connect(refreshTrajectory)

	def loadPath(self):
		self.path = QFileDialog.getOpenFileName(self, 'Open Path File', '',"CERES Path Files (*.path);;All Files (*)")
		refreshTrajectory()
	
	def closeEvent(self, event):
		global procs
		while(len(procs)>0):
			procs[0][1].kill()
			procs.pop(0)
			rospy.logwarn("[GUI] Closed Properly.")
	
	def connectIMU(self):
		global procs
		self.ui.pushButton.setText("Disconnect")
		self.ui.pushButton.clicked.disconnect(self.connectIMU)
		self.ui.pushButton_7.setEnabled(False)
		self.ui.pushButton.clicked.connect(self.disconnectIMU)
		imu = subprocess.Popen(['rosrun', 'advanced_navigation_driver', 'advanced_navigation_driver', '_baud:='+self.ui.comboBox.currentText(), '_port:='+self.ui.comboBox_2.currentText(), '__name:=ceres_IMU']) 
		procs.append(["imu", imu])
		self.ui.label_41.setText("<font color='#00AA00'>Connected</font>")
		self.ui.label_8.setText("<font color='#00AA00'>Connected</font>")

	def disconnectIMU(self):
		global procs
		self.ui.label_41.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.label_8.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.pushButton_7.setEnabled(True)
		self.ui.pushButton.setText("Connect")
		self.ui.pushButton.clicked.disconnect(self.disconnectIMU)
		self.ui.pushButton.clicked.connect(self.connectIMU)
		for i in procs:
			if i[0]=="imu":
				i[1].kill()
				procs.pop(procs.index(i))
				break

	def connectMainArduino(self):
		global procs
		self.ui.pushButton_3.setText("Disconnect")
		self.ui.pushButton_3.clicked.disconnect(self.connectMainArduino)
		self.ui.pushButton_3.clicked.connect(self.disconnectMainArduino)
		self.ui.pushButton_9.setEnabled(False)
		mainArduino = subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', '_port:='+self.ui.comboBox_6.currentText(), '__name:=ceres_MainArduino']) 
		procs.append(["mainArduino", mainArduino])
		self.ui.label_49.setText("<font color='#00AA00'>Connected</font>")
		self.ui.label_10.setText("<font color='#00AA00'>Connected</font>")
		
	def disconnectMainArduino(self):
		global procs
		self.ui.label_49.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.label_10.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.pushButton_9.setEnabled(True)
		self.ui.pushButton_3.setText("Connect")
		self.ui.pushButton_3.clicked.disconnect(self.disconnectMainArduino)
		self.ui.pushButton_3.clicked.connect(self.connectMainArduino)
		for i in procs:
			if i[0]=="mainArduino":
				i[1].kill()
				procs.pop(procs.index(i))
				break
				
	def connectRCArduino(self):
		global procs
		self.ui.pushButton_2.setText("Disconnect")
		self.ui.pushButton_2.clicked.disconnect(self.connectRCArduino)
		self.ui.pushButton_2.clicked.connect(self.disconnectRCArduino)
		self.ui.pushButton_8.setEnabled(False)
		RCArduino = subprocess.Popen(['rosrun', 'rosserial_python', 'serial_node.py', '_port:='+self.ui.comboBox_4.currentText(), '__name:=ceres_ArduinoRC']) 
		procs.append(["RCArduino", RCArduino])
		self.ui.label_45.setText("<font color='#00AA00'>Connected</font>")
		self.ui.label_9.setText("<font color='#00AA00'>Connected</font>")
		
	def disconnectRCArduino(self):
		global procs
		self.ui.label_45.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.label_9.setText("<font color=''#FF0000'>Not Connected</font>")
		self.ui.pushButton_8.setEnabled(True)
		self.ui.pushButton_2.setText("Connect")
		self.ui.pushButton_2.clicked.disconnect(self.disconnectRCArduino)
		self.ui.pushButton_2.clicked.connect(self.connectRCArduino)
		for i in procs:
			if i[0]=="RCArduino":
				i[1].kill()
				procs.pop(procs.index(i))
				break
				
	def startPathGenerator(self):
		global myapp, procs
		flag = True
		for i in procs:
			if i[0]=="PathGenerator" or i[0]=="bagLogger" or i[0]=="csvLogger" or i[0]=="poseController" or i[0]=="PathReader":
				flag=False
				break
		if flag:
			if myapp.ui.horizontalSlider.value()==1:
				csvLogger= subprocess.Popen(['rosrun', 'ceres', 'ceresLog.py', '__name:=ceres_Logger']) 
				procs.append(["csvLogger", csvLogger])
			
			if myapp.ui.horizontalSlider_2.value()==1:
				bagLogger= subprocess.Popen(['rosbag', 'record', '-a', '__name:=ceres_bag']) 
				procs.append(["bagLogger", bagLogger])
		
			if myapp.ui.verticalSlider.value()==1:				
				pathGenerator = subprocess.Popen(['rosrun', 'ceres', 'ceresPathGenerator.py', '1', str(myapp.ui.doubleSpinBox_2.value())]) 
				procs.append(["PathGenerator", pathGenerator])
			elif myapp.ui.verticalSlider.value()==2:
				pathGenerator = subprocess.Popen(['rosrun', 'ceres', 'ceresPathGenerator.py', '0', str(myapp.ui.doubleSpinBox.value())]) 
				procs.append(["PathGenerator", pathGenerator])
			elif myapp.ui.verticalSlider.value()==3:
				poseController = subprocess.Popen(['rosrun', 'ceres', 'ceresController.py', '__name:=ceres_PoseController']) 
				procs.append(["poseController", poseController])
				pathReader = subprocess.Popen(['rosrun', 'ceres', 'ceresPathReader.py', myapp.path, '__name:=ceres_PathReader']) 
				procs.append(["PathReader", pathReader])
		else:
			rospy.logerr("[GUI] A test sequence is already started!")
			
	def stopPathGenerator(self):
		global myapp, procs
		flag=True
		while(flag):
			flag=False
			for i in procs:
				if i[0]=="poseController":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI] Pose Controller Closed.")
					flag=True
					break

				if i[0]=="PathReader":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI] Test aborded.")
					flag=True
					break

				if i[0]=="PathGenerator":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI] Test aborded.")
					flag=True
					break
				if i[0]=="csvLogger":
					i[1].kill()
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI]  CSV data logging finished.")
					flag=True
					break
				if i[0]=="bagLogger":
					temp = subprocess.Popen(['rosnode', 'kill', '/ceres_bag'])
					procs.pop(procs.index(i))
					rospy.logwarn("[GUI]  BAG data logging finished.")
					flag=True
					break
					
	def detectIMU(self):
		global myapp
		
		myapp.ui.comboBox_2.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "USB-RS232 Cable":
				n=i
				break
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.comboBox_2.addItems(L)

	def detectArduinoRC(self):
		global myapp
		
		myapp.ui.comboBox_4.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "USB2.0-Serial":
				n=i
				break
		#L=QStringList()
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.comboBox_4.addItems(L)
		
	def detectMainArduino(self):
		global myapp
		
		myapp.ui.comboBox_6.clear()
		ports = list_ports.comports()
		n=-1
		for i in range(len(ports)):
			if ports[i].description == "Arduino Mega":
				n=i
				break
		#L=QStringList()
		L = []
		if n>-1:
			L.append(ports[n].device)
		for i in range(len(ports)):
			if i!=n:
				L.append(ports[i].device)
		myapp.ui.comboBox_6.addItems(L)

def refreshProcs():
	global procs, myapp
	updateLog()
	for i in procs:
		if i[1].poll() is not None:
			if  i[0] == "imu":
				procs.pop(procs.index(i))
				myapp.ui.label_41.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.label_8.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.pushButton.setText("Connect")
				myapp.ui.pushButton.clicked.disconnect(myapp.disconnectIMU)
				myapp.ui.pushButton.clicked.connect(myapp.connectIMU)
				myapp.ui.pushButton_7.setEnabled(True)
				break
				
			elif  i[0] == "mainArduino":
				procs.pop(procs.index(i))
				myapp.ui.label_49.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.label_10.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.pushButton_3.setText("Connect")
				myapp.ui.pushButton_3.clicked.disconnect(myapp.disconnectMainArduino)
				myapp.ui.pushButton_3.clicked.connect(myapp.connectMainArduino)
				myapp.ui.pushButton_9.setEnabled(True)
				break
				
			elif  i[0] == "RCArduino":
				procs.pop(procs.index(i))
				myapp.ui.label_45.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.label_9.setText("<font color='#FF0000'>Not Connected</font>")
				myapp.ui.pushButton_2.setText("Connect")
				myapp.ui.pushButton_2.clicked.disconnect(myapp.disconnectRCArduino)
				myapp.ui.pushButton_2.clicked.connect(myapp.connectRCArduino)
				myapp.ui.pushButton_8.setEnabled(True)
				break
				
			elif  i[0] == "PathGenerator" or i[0]=="PathReader":
				procs.pop(procs.index(i))
				flag=True
				while(flag):
					flag=False
					for j in procs:
						if j[0]=="poseController":
							j[1].kill()
							procs.pop(procs.index(j))
							rospy.logwarn("[GUI] Pose Controller Closed.")
							flag=True
							break
						if j[0]=="csvLogger":
							j[1].kill()
							procs.pop(procs.index(j))
							rospy.logwarn("[GUI]  CSV data logging finished.")
							flag=True
							break
						if j[0]=="bagLogger":
							temp = subprocess.Popen(['rosnode', 'kill', '/ceres_bag'])
							procs.pop(procs.index(j))
							rospy.logwarn("[GUI]  BAG data logging finished.")
							flag=True
							break
				break				

# callbackRC: called each time a message is received from the RC Arduino.
def callbackRC(data):
	global var
	var[10][3]=data.CH1
	var[11][3]=data.CH2
	var[12][3]=data.emergency
	var[13][3]=data.AUX

# callbackTwist: called each time a message is received from the cmd_vel topic.
def callbackTwist(data):
	global var
	var[8][3]=data.linear.x
	var[9][3]=data.angular.z

# callbackArduino: called each time a message is received from the Arduino.
def callbackArduino(data):
	global var
	var[6][3]=data.Ul
	var[7][3]=data.Ur

 #callBackIMU: called each time the IMU return an Odometry message.
def callbackIMU(data):
	global var
	quaternion = (
	data.pose.pose.orientation.x,
	data.pose.pose.orientation.y,
	data.pose.pose.orientation.z,
	data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	var[3][3] = euler[2] 
	var[18][3] = euler[0] 
	var[19][3] = euler[1] 
	
	var[1][3]=data.pose.pose.position.x
	var[2][3]=data.pose.pose.position.y

	var[4][3]=data.twist.twist.linear.x
	var[17][3]=data.twist.twist.linear.y
	var[5][3]=data.twist.twist.angular.z


 #callBackGPS: called each time the IMU return a GPS Message.
def callbackGPS(data):
	global var
	var[14][3]=data.latitude
	var[15][3]=data.longitude
	var[16][3]=data.altitude

 #callBackLog: called each time ROS returns a message.
def callbackLog(data):
	global logs
	logs.append([data.msg, data.level])

def updateLog():
	global myapp, logs
	while(len(logs)>0):
		if logs[0][1] == 1:
			myapp.ui.textEdit.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#55aa00;\" >"+logs[0][0] +"</span><br/><br/>")
			myapp.ui.textEdit_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#55aa00;\" >"+logs[0][0] +"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)

		elif logs[0][1]  == 2:
			myapp.ui.textEdit.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#0000FF;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.textEdit_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#0000FF;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)
		
		elif logs[0][1]  == 4:
			myapp.ui.textEdit.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#ffaa00;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.textEdit_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#ffaa00;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)

		elif logs[0][1]  == 8:
			myapp.ui.textEdit.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#FF0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.textEdit_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#FF0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)
			
		elif logs[0][1]  == 16:
			myapp.ui.textEdit.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#aa0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.textEdit_2.insertHtml("<span style=\" font-size:8pt; font-weight:600; color:#aa0000;\" >"+logs[0][0]+"</span><br/><br/>")
			myapp.ui.statusBar.showMessage("MainArduino:  "+logs[0][0], 5000)
		logs=logs[1:]
		myapp.ui.textEdit.moveCursor(QTextCursor.End)
		myapp.ui.textEdit_2.moveCursor(QTextCursor.End)
		
# adquire: Initialize all the subscribers/publishers.
def adquire():
	rospy.init_node('Ceres_GUI', anonymous=True)
	rospy.Subscriber("/ceres/arduLog", CeresArduinoLogging, callbackArduino)
	rospy.Subscriber("/ceres/RC", CeresRC, callbackRC)
	rospy.Subscriber("/ceres/cmd_vel", Twist, callbackTwist)
	rospy.Subscriber("/advanced_navigation_driver/odom", Odometry, callbackIMU)
	rospy.Subscriber("/advanced_navigation_driver/nav_sat_fix", NavSatFix, callbackGPS)
	rospy.Subscriber("/rosout", Log, callbackLog)

def refreshGraph():
	global myapp, var, eastingList, northingList, headingList, velocityXList, velocityYList, angularVelocityList, timeList, graphRate
	
	timeList.pop(0)
	timeList.append(timeList[-1]+1.0/graphRate)
	eastingList.pop(0)
	eastingList.append(var[1][3])
	myapp.ui.graphicsView_8.clear()
	myapp.ui.graphicsView_8.plot(timeList,eastingList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.graphicsView_8.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.graphicsView_8.showGrid(x=True, y=True)

	northingList.pop(0)
	northingList.append(var[2][3])
	myapp.ui.graphicsView_7.clear()
	myapp.ui.graphicsView_7.plot(timeList,northingList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.graphicsView_7.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.graphicsView_7.showGrid(x=True, y=True)

	headingList.pop(0)
	headingList.append(360*var[3][3]/(2*3.14))
	myapp.ui.graphicsView_6.clear()
	myapp.ui.graphicsView_6.plot(timeList,headingList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.graphicsView_6.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.graphicsView_6.showGrid(x=True, y=True)

	velocityXList.pop(0)
	velocityXList.append(var[4][3])
	myapp.ui.graphicsView_9.clear()
	myapp.ui.graphicsView_9.plot(timeList,velocityXList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.graphicsView_9.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.graphicsView_9.showGrid(x=True, y=True)

	velocityYList.pop(0)
	velocityYList.append(var[17][3])
	myapp.ui.graphicsView_10.clear()
	myapp.ui.graphicsView_10.plot(timeList,velocityYList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.graphicsView_10.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.graphicsView_10.showGrid(x=True, y=True)

	angularVelocityList.pop(0)
	angularVelocityList.append(var[5][3])
	myapp.ui.graphicsView_11.clear()
	myapp.ui.graphicsView_11.plot(timeList,angularVelocityList, pen=pyqtgraph.mkPen('k', width=3))
	myapp.ui.graphicsView_11.setXRange(timeList[0], timeList[-1], padding=0)
	myapp.ui.graphicsView_11.showGrid(x=True, y=True)
	

def refreshData():
	global myapp, var, i
	myapp.ui.label_27.setText(str(round(var[14][3], 8))) #Refresh Latitude
	myapp.ui.label_28.setText(str(round(var[15][3], 8))) #Refresh Longitude
	myapp.ui.label_31.setText(str(round(var[16][3], 0))) #Refresh Altitude
	myapp.ui.label_29.setText(str(round(var[1][3], 2))) #Refresh Easting
	myapp.ui.label_30.setText(str(round(var[2][3], 2))) #Refresh Northing

	myapp.ui.progressBarThrottle.setValue(var[10][3]) #Refresh CH1
	myapp.ui.progressBarYaw.setValue(var[11][3]) #Refresh CH2
	myapp.ui.progressBarAUX.setValue(var[13][3]) #Refresh AUX
	myapp.ui.progressBarAU.setValue(var[12][3]) #Refresh AU
	
	myapp.ui.progressBar.setValue(int(1000*var[6][3])) #Refresh Voltage
	myapp.ui.progressBar_2.setValue(int(1000*var[7][3])) #Refresh Voltage
	
	if var[12][3] > 50:
		myapp.ui.label_12.setText("<font color='#FF0000'>Emergency Stop</font>")
	elif var[13][3] > 50:
		myapp.ui.label_12.setText("<font color='#00AA00'>Automatic Mode</font>")
	else:
		myapp.ui.label_12.setText("<font color='#ffaa00'>Manual Mode</font>")
	
	if(var[14][3]!=0.0 or var[15][3] != 0.0):
		coords = var[14][3], var[15][3]
		myapp.ui.graphicsView_3.addMarker("Position Time: "+str(var[0][3]), *coords, **dict(
			icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_red.png",
			draggable=False,
			title="Position Time: "+str(var[0][3])
		))
		myapp.ui.graphicsView_3.centerAt(var[14][3], var[15][3])

		coords = var[14][3], var[15][3]
		myapp.ui.graphicsView.addMarker("Position Time: "+str(var[0][3]), *coords, **dict(
			icon="http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_red.png",
			draggable=False,
			title="Position Time: "+str(var[0][3])
		))
		myapp.ui.graphicsView.centerAt(var[14][3], var[15][3])

	img = QImage()
	path = os.path.dirname(os.path.abspath(__file__))
	img.load(os.path.join(path, 'heading.png'))
	pixmap = QPixmap(img)
	transform = QTransform().rotate(-(var[3][3]*180.0/3.14)+90.0)
	pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
	myapp.ui.label_72.setPixmap(pixmap)
	
	img = QImage()
	img.load(os.path.join(path, 'roll.png'))
	pixmap = QPixmap(img)
	transform = QTransform().rotate((var[18][3]*180/3.14+180))
	pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
	myapp.ui.label_73.setPixmap(pixmap)
	
	img = QImage()
	img.load(os.path.join(path, 'pitch.png'))
	pixmap = QPixmap(img)
	transform = QTransform().rotate((var[19][3]*180.0/3.14))
	pixmap = pixmap.transformed(transform, Qt.SmoothTransformation)
	myapp.ui.label_74.setPixmap(pixmap)
	
	myapp.ui.label_75.setText(str(round(var[3][3]*180.0/3.14,1))+" deg")
	myapp.ui.label_76.setText(str((round(var[18][3]*180.0/3.14,1)+180.0)%360)+" deg")
	myapp.ui.label_77.setText(str(round(-var[19][3]*180.0/3.14,1))+" deg")

		
		
def refreshTrajectory():
	global myapp
	if  myapp.ui.verticalSlider.value()==1:
		myapp.ui.pushButton_5.setEnabled(True)
		points = calculateCirclePoints(myapp.ui.doubleSpinBox_2.value())
	elif myapp.ui.verticalSlider.value()==2:
		myapp.ui.pushButton_5.setEnabled(True)
		points=calculateLinePoints(myapp.ui.doubleSpinBox.value())
	elif myapp.ui.verticalSlider.value()==3:
		if(myapp.path ==""):
			myapp.ui.pushButton_5.setEnabled(False)
			rospy.logwarn("[GUI]  Please load a Path File.")
			points=[[0],[0]]
		else:
			myapp.ui.pushButton_5.setEnabled(True)
			points=readPath()
	else:
		points=[[0.0],[0.0]]
	myapp.ui.graphicsView_13.clear()
	if myapp.ui.verticalSlider.value()==3:
		myapp.ui.graphicsView_13.plot(points[0], points[1], pen=pyqtgraph.mkPen('r', width=3), symbol='d')
	else:
		myapp.ui.graphicsView_13.plot(points[0], points[1], pen=pyqtgraph.mkPen('r', width=3))

	path = os.path.dirname(os.path.abspath(__file__))
	img = QtWidgets.QGraphicsPixmapItem(QtGui.QPixmap(os.path.join(path, 'top.png')))
	img.scale(0.04,-0.04)
	img.translate(-2.02/0.04,-0.82/0.04)
	myapp.ui.graphicsView_13.addItem(img)
	lims = myapp.ui.graphicsView_13.getViewBox().childrenBounds()
	mini = min(lims[0][0], lims[1][0])
	maxi = max(lims[0][1], lims[1][1])
	myapp.ui.graphicsView_13.setXRange(mini-0.5, maxi+0.5)
	myapp.ui.graphicsView_13.setYRange(mini-0.5, maxi+0.5)

	myapp.ui.graphicsView_13.setLabel('left', "Robot Front Axis")
	myapp.ui.graphicsView_13.setLabel('bottom', "Robot Right Axis")

def readPath():
	global myapp, var
	file=open(myapp.path, "r")
	l=file.readline()
	origins=[-1.0,-1.0,-1.0] # Origin: Lat, Long, Psi
	originsFlag=[False,False,False]
	points=[] # [[X, Y, Psi]]

	# Decrypt .path file
	while(l!=""):
		elements=l.replace("\n", "").replace("\r","").split(" ")
		if elements[0]=="POSITION":
			if elements[1]!="RELATIVE":
				originsFlag[0]=True
				originsFlag[1]=True
				origins[0]=float(elements[1])
				origins[1]=float(elements[2])
		elif elements[0]=="ORIENTATION":
			if elements[1]!="RELATIVE":
				originsFlag[2]=True
				origins[2]=float(elements[1])/180.*3.14

		elif elements[0][0]=="X":
			points.append([float(elements[0][1:]), float(elements[1][1:])])

		l=file.readline()
	output=[[],[]]
	if originsFlag[0]==False:
		dX = 0
		dY = 0
	else:
		robotPose = utm.from_latlon(var[14][3], var[15][3])
		pathPose = utm.from_latlon(origins[0], origins[1])
		dXi = (pathPose[0]-robotPose[0])
		dYi = (pathPose[1]-robotPose[1])
		dX = dXi*cos(-var[3][3]) - dYi*sin(-var[3][3])
		dY = dYi*cos(-var[3][3]) + dXi*sin(-var[3][3])
		if sqrt(pow(dX,2)+pow(dY,2))>10:
			rospy.logwarn("[GUI] The robot is far from the Path Origin, PathReader may refuse Path file!")

	if originsFlag[2]==False:
		dAngle = 0
	else:
		dAngle = (origins[2]-var[3][3])# Calcul IMU Angle
	for i in points:
		X = i[0]
		Y = i[1]
		output[0].append(-(Y*cos(dAngle)+X*sin(dAngle)+dY)) # Xr
		output[1].append(X*cos(dAngle)-Y*sin(dAngle)+dX) # Yr
	return output

def calculateLinePoints(length):
	X=[]
	Y=[]
	for i in range(1000):
		X.append(0)
		Y.append(length*i/1000)
	return [X,Y]
def calculateCirclePoints(diameter):
	X=[]
	Y=[]
	for i in range(1000):
		X.append(diameter/2.0*cos(i*2.0*3.14/1000.0)-diameter/2)
		Y.append(diameter/2.0*sin(i*2.0*3.14/1000.0))
	return [X,Y]
		
		
if __name__ == "__main__":
	global var, myapp, i
	global eastingList, northingList, headingList, velocityXList, velocityYList, angularVelocityList, timeList, graphRate
	global procs

	pyqtgraph.setConfigOption('background', [247,246,246,255])
	pyqtgraph.setConfigOption('foreground', 'k')

	procs=[]
	i=0
	rate = 20 # Recording Frequency
	graphRate = 1 # Graph Refreshing Rate
	procsRate = 50 # External processes refreshing rate for reading the terminal messages

	logs = []
	
	var=[]
	var.append(["time","Time", "[s]", 0, False])				#0
	var.append(["Easting", "Robot Position East Axis", "[m]", 0, False]) 	#1
	var.append(["Northing", "Robot Position North Axis", "[m]", 0, False])	#2
	var.append(["heading", "Robot Heading to North", "[rad]", 0.0, False])	#3
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
	var.append(["Roll", "Robot Roll", "[rad]", 3.14, False])			#18
	var.append(["Pitch", "Robot Pitch", "[rad]", 0.0, False])			#19


	timeList=[i*(1.0/graphRate) for i in range(60)]
	eastingList=[0 for i in range(60)]
	northingList=[0 for i in range(60)]
	headingList=[0 for i in range(60)]
	velocityXList=[0 for i in range(60)]
	velocityYList=[0 for i in range(60)]
	angularVelocityList=[0 for i in range(60)]

	app = QtWidgets.QApplication(sys.argv)
	myapp = CeresApplication()
	
	timer = QTimer()

	graphTimer = QTimer()
	procsTimer = QTimer()

	timeInit=time.time()
	
	adquire()
	
	timer.timeout.connect(refreshData)
	timer.start(int(1000/rate))

	graphTimer.timeout.connect(refreshGraph)
	graphTimer.start(int(1000/graphRate))

	procsTimer.timeout.connect(refreshProcs)
	procsTimer.start(int(1000/procsRate))
	
	myapp.ui.graphicsView.waitUntilReady()
	myapp.ui.graphicsView.centerAt(4.638, -74.08523)
	myapp.ui.graphicsView.setZoom(15)
	
	myapp.ui.graphicsView_3.waitUntilReady()
	myapp.ui.graphicsView_3.centerAt(4.638, -74.08523)
	myapp.ui.graphicsView_3.setZoom(16)

	myapp.ui.graphicsView_13.setLabel('left', "Robot Front Axis")
	myapp.ui.graphicsView_13.setLabel('bottom', "Robot Right Axis")
	myapp.ui.graphicsView_13.showGrid(x=True, y=True)
	path = os.path.dirname(os.path.abspath(__file__))
	img = QPixmap('top.png')
	#img.scaled(Qt.IgnoreAspectRatio)
	pixItem = QtWidgets.QGraphicsPixmapItem(img)
	# img = pyqtgraph.QtWidgets.QGraphicsPixmapItem(pyqtgraph.QtWidgets.QPixmap(os.path.join(path, 'top.png')))
	#img.scale(0.04,-0.04)
	#img.translate(-2.02/0.04,-0.82/0.04)
	myapp.ui.graphicsView_13.addItem(pixItem)
	
	myapp.show()
	sys.exit(app.exec_())

