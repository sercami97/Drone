#!/usr/bin/env python3
# coding=utf-8

#import os
#os._exit(00)


import rospy
# from pynput.keyboard import Key, Listener
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String, Int32

import time
from time import sleep
import math

import serial as pyserial

from json       import dumps
from os         import system
from WiiProxy   import MultiWii


data = [1000,1000,1000,1000,1000,1000,1000,1000]
aux1 = 1000
aux2 = 1000
estado = 0

m = Float32MultiArray()

controller  = None
serial      = None

IMU = []
motorsPWM = []

estado_str = ""



def initSerial():
    global serial, controller
    # -----------------------------------------------------------

    serial = pyserial.Serial()

    serial.port             = "/dev/ttyUSB0"
    serial.baudrate         = 115200
    serial.bytesize         = pyserial.EIGHTBITS
    serial.parity           = pyserial.PARITY_NONE
    serial.stopbits         = pyserial.STOPBITS_ONE
    serial.write_timeout    = 3
    serial.xonxoff          = False
    serial.rtscts           = False
    serial.dsrdtr           = False

    serial.open()

    sleep(6)

    controller = MultiWii(serial)
def sendSerial(msg):
    global controller, aux1, aux2, estado
    print(msg)
    channels = [int(msg[0]),int(msg[1]),int(msg[2]),int(msg[3]),aux1,aux2,0,0] 
    controller.set_channels(channels)
    #print(channels)
def calibrate():
    global controller
    controller.calibrate_acc()
def sensores():
    global estado
    if not(estado==2):
        #motorsPWM = controller.get_motors()
        #attitude = controller.get_attitude()
        voltage = controller.get_voltage()
        #m.data = [attitude['angx'],attitude['angy'],attitude['heading'],motorsPWM[0],motorsPWM[1],motorsPWM[2],motorsPWM[3],voltage[0],estado]
        m.data = [voltage[0],voltage[1],voltage[0],voltage[0],voltage[0],voltage[0],voltage[0], voltage[0],estado]
        #m.data = [motorsPWM[0],motorsPWM[0],motorsPWM[0],motorsPWM[0],motorsPWM[1],motorsPWM[2],motorsPWM[3],motorsPWM[3],estado]
        #print("Voltage: ", voltage[0])

def callBackPrueba(msg):
    global data, estado
    data = msg.data
    print("Callback")
    #print(msg.data)
    
    estado = data[4]
    print("Estado ", estado)
    

#Método constructor. Se establecen las conexion y se inicializan variables.
def init():
    global data, estado, aux1, aux2

    rospy.init_node('RecibirMov', anonymous=True)
    pub = rospy.Publisher('sensores', Float32MultiArray, queue_size = 100)
    rospy.Subscriber('movs', Float32MultiArray, callBackPrueba)#Control

    print("InitScript")
    try:
        initSerial()
        rate = rospy.Rate(10)
        print("Velocidad")
        m.data = [0,0]
        aux1 = 0
        aux2 = 0
        sendSerial(data)
        #sleep(5)
        ar = 0
        
        start_time = time.time()
        calibration_over = False
        while not rospy.is_shutdown():
            elapsed = start_time - time.time()
            print("While ", elapsed)
            start_time = time.time()
            if(estado==1):
                print("Armed")
                aux1 = 1900
                aux2 = 1000
            elif(estado==2 and calibration_over==False):
                print("Calibrate")
                calibrate()
                sleep(1)
                calibration_over = True
            elif(estado==3):
                print("Calibrate")
                aux2 = 1900
                aux1 = 1000
            else:
                print("Disarmed")
                calibration_over = False
                aux1 = 1000
                aux2 = 1000
            print("Send")    
            sendSerial(data)
            sensores()
            
            pub.publish(m)
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
