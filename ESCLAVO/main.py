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


#Método constructor. Se establecen las conexion y se inicializan variables.
def init():

    pub = rospy.Publisher('vueltas', Float32MultiArray, queue_size = 100)
    rospy.init_node('Encoder', anonymous=True)

    print("probando2222")
    try:
        rate = rospy.Rate(10)
        print("Velocidad")
        m.data = [0,0]

        while not rospy.is_shutdown():
            m.data = [1,1]
            pub.publish(m)
            print("Publicar")
            print(m)
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