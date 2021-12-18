#!/usr/bin/env python
# license
import sys
import rospy
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from mainwindow import Ui_MainWindow
import pyqtgraph as pg
import TabCeres
from math import sin, cos

class ShipHolderApplication(QMainWindow):
	def __init__(self, parent=None):
		super (ShipHolderApplication, self).__init__(parent)
		self.createWidgets()

	def createWidgets(self):
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.ui.tabs = []
		self.ui.actionOpen_CERES_Log_File.triggered.connect(self.loadPath)
		self.ui.tabWidget.removeTab(0)
		QObject.connect(self.ui.tabWidget, SIGNAL('tabCloseRequested(int)'), self.closeTab)
		QObject.connect(self.ui.pushButton, SIGNAL("clicked()"), self.loadPath)

	def closeTab(self, num):
		self.ui.tabWidget.removeTab(num)

	def loadPath(self):
		self.addTab(QFileDialog.getOpenFileName(self, 'Open Log File', '',"CERES CSV Files (*.csv);;All Files (*)"))
	
	def addTab(self, path):
		file = open(path, "r")
		line = file.readline()
		data = [[],[],[],[],[],[],[],[],[],[],[],[]]
		pidU=["NaN","NaN","NaN"]
		pidW=["NaN","NaN","NaN"]
		controlPose=["NaN","NaN","NaN"]
		while line!="" and (len(line.replace("\r", "").replace("\n", "").split(";"))>22 or line.split(";")[0][0:3]=="KpU"):
			temp = line.replace("\r", "").replace("\n", "").replace(",",".").split(";")
			if temp[0][0:3]=="KpU":
				pidU[0] = temp[0][3:]
				pidU[1] = temp[1][3:]
				pidU[2] = temp[2][3:]

				pidW[0] = temp[3][3:]
				pidW[1] = temp[4][3:]
				pidW[2] = temp[5][3:]

				controlPose[0] = temp[6][3:]
				controlPose[1] = temp[7][3:]
				controlPose[2] = temp[8][3:]
			else:
				try:
					if float(temp[20])!=0 or float(temp[21])!=0 or float(temp[22])!=0:
						data[0].append(float(temp[0]))

						data[1].append(float(temp[1]))
						data[2].append(float(temp[20]))

						data[3].append(float(temp[2]))
						data[4].append(float(temp[21]))

						data[5].append(float(temp[3])/3.14*180.)
						data[6].append(float(temp[22])/3.14*180.)
					
						ErrX = data[2][-1] - data[1][-1]
						ErrY = data[4][-1] - data[3][-1]
						data[7].append(ErrX * cos(float(temp[3])) + ErrY * sin(float(temp[3])))
						data[8].append(ErrY * cos(float(temp[3])) - ErrX * sin(float(temp[3])))
						data[9].append(data[6][-1] - data[5][-1])
				except ValueError:
					pass
			line = file.readline()


		newTab = TabCeres.TabCeres()
		self.ui.tabs.append(newTab)

		self.ui.tabs[-1].graphicsView.plot(data[1], data[3], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView.plot(data[2], data[4], pen=pg.mkPen('r', width=2))
		self.ui.tabs[-1].graphicsView.setLabel('bottom', "Easting [m]")
		self.ui.tabs[-1].graphicsView.setLabel('left', "Northing [m]")
		self.ui.tabs[-1].graphicsView.showGrid(x=True, y=True)
		minX = min(min(data[1]),min(data[2]))
		maxX = max(max(data[1]),max(data[2]))
		minY = min(min(data[3]),min(data[4]))
		maxY = max(max(data[3]),max(data[4]))

		dX = maxX-minX
		dY = maxY-minY
		d = max(dX,dY)/2
		mX = (maxX+minX)/2
		mY = (maxY+minY)/2
	
		self.ui.tabs[-1].graphicsView.setXRange(mX-d, mX+d)
		self.ui.tabs[-1].graphicsView.setYRange(mY-d, mY+d)

		self.ui.tabs[-1].graphicsView_2.plot(data[0], data[1], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView_2.plot(data[0], data[2], pen=pg.mkPen('r', width=2))
		self.ui.tabs[-1].graphicsView_2.setLabel('bottom', "Time [s]")
		self.ui.tabs[-1].graphicsView_2.setLabel('left', "Easting [m]")
		self.ui.tabs[-1].graphicsView_2.showGrid(x=True, y=True)

		self.ui.tabs[-1].graphicsView_4.plot(data[0], data[3], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView_4.plot(data[0], data[4], pen=pg.mkPen('r', width=2))
		self.ui.tabs[-1].graphicsView_4.setLabel('bottom', "Time [s]")
		self.ui.tabs[-1].graphicsView_4.setLabel('left', "Northing [m]")
		self.ui.tabs[-1].graphicsView_4.showGrid(x=True, y=True)

		self.ui.tabs[-1].graphicsView_3.plot(data[0], data[5], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView_3.plot(data[0], data[6], pen=pg.mkPen('r', width=2))
		self.ui.tabs[-1].graphicsView_3.setLabel('bottom', "Time [s]")
		self.ui.tabs[-1].graphicsView_3.setLabel('left', "Heading [deg]")
		self.ui.tabs[-1].graphicsView_3.showGrid(x=True, y=True)

		self.ui.tabs[-1].label_20.setText(pidU[0])
		self.ui.tabs[-1].label_18.setText(pidU[1])
		self.ui.tabs[-1].label_15.setText(pidU[2])

		self.ui.tabs[-1].label_17.setText(pidW[0])
		self.ui.tabs[-1].label_19.setText(pidW[1])
		self.ui.tabs[-1].label_16.setText(pidW[2])

		self.ui.tabs[-1].label_23.setText(controlPose[0])
		self.ui.tabs[-1].label_22.setText(controlPose[1])
		self.ui.tabs[-1].label_21.setText(controlPose[2])

		self.ui.tabs[-1].label_3.setText("Parameters")
		self.ui.tabs[-1].label_11.setText("Err Kd")
		self.ui.tabs[-1].label_14.setText("X Kp")
		self.ui.tabs[-1].label_7.setText("Linear Kd")
		self.ui.tabs[-1].label_5.setText("Linear Kp")
		self.ui.tabs[-1].label_10.setText("Angular Kp")
		self.ui.tabs[-1].label_9.setText("Angular Ki")
		self.ui.tabs[-1].label_6.setText("Linear Ki")
		self.ui.tabs[-1].label_4.setText("MainArduino cmd_vel PID")
		self.ui.tabs[-1].label_13.setText("ROS cmd_pose Controller")
		self.ui.tabs[-1].label_8.setText("Angular Kd")
		self.ui.tabs[-1].label_12.setText("Y Ki")

        	self.ui.tabWidget.addTab(self.ui.tabs[-1].tab, path.split("/")[-1]+" - Inertial")

		##---

		newTab = TabCeres.TabCeres()
		self.ui.tabs.append(newTab)

		self.ui.tabs[-1].graphicsView.plot(data[1], data[3], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView.plot(data[2], data[4], pen=pg.mkPen('r', width=2))
		self.ui.tabs[-1].graphicsView.setLabel('bottom', "Easting [m]")
		self.ui.tabs[-1].graphicsView.setLabel('left', "Northing [m]")
		self.ui.tabs[-1].graphicsView.showGrid(x=True, y=True)
		minX = min(min(data[1]),min(data[2]))
		maxX = max(max(data[1]),max(data[2]))
		minY = min(min(data[3]),min(data[4]))
		maxY = max(max(data[3]),max(data[4]))

		dX = maxX-minX
		dY = maxY-minY
		d = max(dX,dY)/2
		mX = (maxX+minX)/2
		mY = (maxY+minY)/2
	
		self.ui.tabs[-1].graphicsView.setXRange(mX-d, mX+d)
		self.ui.tabs[-1].graphicsView.setYRange(mY-d, mY+d)

		self.ui.tabs[-1].graphicsView_2.plot(data[0], data[7], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView_2.setLabel('bottom', "Time [s]")
		self.ui.tabs[-1].graphicsView_2.setLabel('left', "Robot X Error [m]")
		self.ui.tabs[-1].graphicsView_2.showGrid(x=True, y=True)

		self.ui.tabs[-1].graphicsView_4.plot(data[0], data[8], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView_4.setLabel('bottom', "Time [s]")
		self.ui.tabs[-1].graphicsView_4.setLabel('left', "Robot Y Error [m]")
		self.ui.tabs[-1].graphicsView_4.showGrid(x=True, y=True)

		self.ui.tabs[-1].graphicsView_3.plot(data[0], data[9], pen=pg.mkPen('b', width=2))
		self.ui.tabs[-1].graphicsView_3.setLabel('bottom', "Time [s]")
		self.ui.tabs[-1].graphicsView_3.setLabel('left', "Robot Psi Error [deg]")
		self.ui.tabs[-1].graphicsView_3.showGrid(x=True, y=True)

		self.ui.tabs[-1].label_20.setText(pidU[0])
		self.ui.tabs[-1].label_18.setText(pidU[1])
		self.ui.tabs[-1].label_15.setText(pidU[2])

		self.ui.tabs[-1].label_17.setText(pidW[0])
		self.ui.tabs[-1].label_19.setText(pidW[1])
		self.ui.tabs[-1].label_16.setText(pidW[2])

		self.ui.tabs[-1].label_23.setText(controlPose[0])
		self.ui.tabs[-1].label_22.setText(controlPose[1])
		self.ui.tabs[-1].label_21.setText(controlPose[2])
		self.ui.tabs[-1].label_3.setText("Parameters")
		self.ui.tabs[-1].label_11.setText("Err Kd")
		self.ui.tabs[-1].label_14.setText("X Kp")
		self.ui.tabs[-1].label_7.setText("Linear Kd")
		self.ui.tabs[-1].label_5.setText("Linear Kp")
		self.ui.tabs[-1].label_10.setText("Angular Kp")
		self.ui.tabs[-1].label_9.setText("Angular Ki")
		self.ui.tabs[-1].label_6.setText("Linear Ki")
		self.ui.tabs[-1].label_4.setText("MainArduino cmd_vel PID")
		self.ui.tabs[-1].label_13.setText("ROS cmd_pose Controller")
		self.ui.tabs[-1].label_8.setText("Angular Kd")
		self.ui.tabs[-1].label_12.setText("Y Ki")

        	self.ui.tabWidget.addTab(self.ui.tabs[-1].tab, path.split("/")[-1]+" - Mobile")

if __name__ == "__main__":
	global myapp
	pg.setConfigOption('background', [247,246,246])
	pg.setConfigOption('foreground', 'k')


	app = QApplication(sys.argv)
	myapp = ShipHolderApplication()
	
	myapp.show()
	sys.exit(app.exec_())

