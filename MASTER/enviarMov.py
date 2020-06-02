#!/usr/bin/env python3
# coding=utf-8
import rospy
# from pynput.keyboard import Key, Listener
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String, Int32

import time
from time import sleep 
import math

import sys
sys.path.insert(1, '~/home/user/catkin_ws/src/drone/scripts')

import xbox

joy = xbox.Joystick()         #Initialize xbox controller


m = Float32MultiArray()
estado = 0


#Método constructor. Se establecen las conexion y se inicializan variables.
def init():

    global m, estado, acumU, acumD, sumThrot

    pub = rospy.Publisher('movs', Float32MultiArray, queue_size = 100)
    rospy.init_node('EnviarMov', anonymous=True)

    try:
        rate = rospy.Rate(10)
        print("Velocidad")
        m.data = [0,0]
        acumD = False
        acumU = False
        sumThrot = 0

        while not rospy.is_shutdown():
            readControl()
            pub.publish(m)
            # print("Publicar")
            # print(m)
            rate.sleep()

    except rospy.ServiceException as e:
        print("Error in Main!")

def readControl():
    global m, estado, acumU, acumD, sumThrot

    (xL,yL) = joy.leftStick()
    (xR,yR) = joy.rightStick()
    triggerR  = joy.rightTrigger()
    triggerL  = joy.leftTrigger()
    sumRoll = (int)(xL*200)
    sumPitch = (int)(yL*200)
    sumYaw = (int)((triggerR - triggerL)*100)
    if(joy.dpadDown()==1 and acumD==False):
        print("Wiii")
        sumThrot -= 50
        acumD = True
    elif(joy.dpadDown()==1 and acumD==True):
        acumD = False
    elif(joy.dpadUp()==1 and acumU==False):
        print("Wiii")
        sumThrot += 50
        acumU = True
    elif(joy.dpadUp()==1 and acumU==True):
        acumU = False
    # roll/pitch/yaw/throt
    roll = 1500+sumRoll
    pitch = 1500+sumPitch
    yaw = 1500+sumYaw
    throtlle = 1000+sumThrot

    if(joy.A()):
        estado = 1
    if(joy.B()):
        estado = 0
    if(joy.X()):
        estado = 2
    if(joy.Y()):
        estado = 3

    if(estado==0):
        throtlle = 1000
        sumThrot = 50

    m.data = [roll,pitch, yaw, throtlle, estado] 
#Primer método en ser llamado por default
if __name__ == '__main__':
    try:
	    init()
    except rospy.ROSInterruptException:
        print("Chaooo")
pass

