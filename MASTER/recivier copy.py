#!/usr/bin/env python
# coding=utf-8
import rospy
# from pynput.keyboard import Key, Listener
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String, Int32

import time
from time import sleep 
import math

m = Float32MultiArray()

def callBackPrueba(msg):
    print("Callback")
    print(msg.data[0])

#Método constructor. Se establecen las conexion y se inicializan variables.
def init():

    rospy.init_node('Encoder', anonymous=True)
    rospy.Subscriber('vueltas', Float32MultiArray, callBackPrueba)#Control

    print("InitScript")
    try:
        rate = rospy.Rate(10)
        print("Velocidad")
        m.data = [0,0]

        while not rospy.is_shutdown():
            print("While")
            rate.sleep()

    except rospy.ServiceException as e:
        print("Error! Make sure pacman world node is running")


#Primer método en ser llamado por default
if __name__ == '__main__':
    try:
	    init()
    except rospy.ROSInterruptException:
		print("Chaooo")
pass

