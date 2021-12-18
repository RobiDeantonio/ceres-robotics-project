#!/usr/bin/env python
# license


from Tkinter import *
import rospy
from ceres.msg import CeresRC

state_AUX1=False
state_AUX2=False

def callback_AUX1():
	global state_AUX1, AUX1
	if state_AUX1:
		AUX1.config(relief="raised")
		state_AUX1=False
	else:
		AUX1.config(relief="sunken")
		state_AUX1=True

	sendData()

def callback_AUX2():
	global state_AUX2, AUX2
        if state_AUX2:
                AUX2.config(relief="raised")
                state_AUX2=False
        else:
                AUX2.config(relief="sunken")
                state_AUX2=True
	sendData()

def sendData():
	global state_AUX1, state_AUX2,pub
	msg=CeresRC()
	msg.emergency=int(state_AUX1)*100
	msg.AUX=int(state_AUX2)*100
	pub.publish(msg)

win = Tk()
win.title("CERES - RADIO CONTROL SIMULATOR")
win.resizable(0,0)

auxFrame=Frame(win,borderwidth=2, relief=GROOVE)
auxFrame.pack(side=TOP, padx=15, pady=30)

AUX1 = Button(auxFrame, text='EMERGENCY', relief=RAISED, command=callback_AUX1)
AUX1.pack(side=RIGHT)

AUX2 = Button(auxFrame, text='MODE', relief=RAISED, command=callback_AUX2)
AUX2.pack(side=LEFT)



leftStick = Frame(win, borderwidth=2, relief=GROOVE)
leftStick.pack(side=LEFT, padx=15, pady=30)

throttle = DoubleVar()
scale = Scale(leftStick, orient='vertical', from_=100, to=0, resolution=1, tickinterval=25, length=200, label='Throttle', variable=throttle)
scale.pack()



rightStick = Frame(win, borderwidth=2, relief=GROOVE)
rightStick.pack(side=RIGHT, padx=15, pady=30)

angular = DoubleVar()
scale1 = Scale(rightStick, orient='horizontal', from_=-100, to=100, resolution=2, tickinterval=50, length=200, label='Angular Velocity', variable=angular)
scale1.pack()


pub = rospy.Publisher('/ceres/RC', CeresRC, queue_size=10)
rospy.init_node('ceres_RadioSimulator')


win.mainloop()
