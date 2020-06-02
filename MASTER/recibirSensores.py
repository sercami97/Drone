#!/usr/bin/env python3
# coding=utf-8
import rospy
# from pynput.keyboard import Key, Listener
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String, Int32

import time
from time import sleep 

import pygame
import math
import numpy

from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

useQuat = False   # set true for using quaternions, false for using y,p,r angles
gyr = [0.0,0.0,0.0]
motors = [0,0,0,0]

control = [1000,1000,1000,1000]

voltage = 0
estado = "Desarmado"

m = Float32MultiArray()

def callBackSensores(msg):
    global gyr, voltage, motors
    print("Callback ", msg.data)
    
    gyr[0] = msg.data[0]
    gyr[1] = (-1)*msg.data[1]
    gyr[2] = (-1)*msg.data[2]

    motors[0] = msg.data[3]
    motors[1] = msg.data[4]
    motors[2] = msg.data[5]
    motors[3] = msg.data[6]

    voltage = msg.data[7]/10

    print("Voltage: ", voltage)

def callBackControl(msg):
    global control, estado
    print("Callback ", msg.data)

    control = [msg.data[0],msg.data[1],msg.data[2],msg.data[3]]
    
    val = msg.data[4]
    if(val==1):
        estado = "Armado"
    if(val==2):
        estado = "Calibrating"
    if(val==3):
        estado = "Horizon"
    if(val==0):
        estado = "Desarmado2"

    print("Voltage: ", voltage)

#Método constructor. Se establecen las conexion y se inicializan variables.
def init():
    global frames
    rospy.init_node('SensoresRecibir', anonymous=True)
    rospy.Subscriber('sensores', Float32MultiArray, callBackSensores)#Control
    rospy.Subscriber('movs', Float32MultiArray, callBackControl)#Control

    initGame()
    
    
    try:
        rate = rospy.Rate(10)
        print("Velocidad")
        m.data = [0,0]

        while not rospy.is_shutdown():
            event = pygame.event.poll()
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                break

            [yaw, pitch, roll] = read_data()
            draw(1, yaw, pitch, roll)
            pygame.display.flip()
            frames += 1
            rate.sleep()

    except rospy.ServiceException as e:
        print("Error! Make sure pacman world node is running")


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def initGame():
    global frames
    width = 800
    height = 600
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((width, height), video_flags)
    pygame.display.set_caption("PyTeapot IMU orientation visualization")
    resizewin(width, height)
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    frames = 0
    ticks = pygame.time.get_ticks()


def read_data():
    global gyr
            
    yaw = float(gyr[2])
    pitch = float(gyr[1])
    roll = float(gyr[0])
    return [yaw, pitch, roll]


def draw(w, nx, ny, nz):
    global voltage, motors, estado
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0.0, 0.0, -10.0)

    drawText((-1, 3.7, 0), "Control Pi Drone", 20)
    drawText((-4.4, 1.6, 0), "Estado: %s" %(estado), 16)
    drawText((-4.4, 1.2, 0), "Voltaje: %.2f" %(voltage), 16)

    # drawText((-2.6, -1.6, 2), "Press Escape to exit.", 16)

    yaw = nx
    pitch = ny
    roll = nz
    # drawText((-1.6, -1.8, 2), "Yaw2: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)


    glPushMatrix()
    glTranslatef(3, -1.5, 0)
    glRotatef(-roll, 0.00, 0.00, 1.00)
    glRotatef(pitch, 1.00, 0.00, 0.00)
    glRotatef(yaw, 0.00, 1.00, 0.00)
    glBegin(GL_QUADS)
    
    max_val = 1
    h = 0.2
    vertices = [
        (max_val , -h, -max_val),
        (max_val , h, -max_val),
        (-max_val , h, -max_val),
        (-max_val , -h, -max_val),
        (max_val , -h, max_val),
        (max_val , h, max_val),
        (-max_val , -h, max_val),
        (-max_val , h, max_val),

    ]

    surfaces = [
        (1,2,3,4),
        (3,2,7,6),
        (6,7,5,4),
        (4,5,1,0),
        (1,5,7,2),
        (4,0,3,6),
    ]

    colors = [
        (0,1,0),
        (1,0.5,0),
        (1,0,0),
        (1,1,0),
        (0,0,1),
        (1,0,1),
    ]
    
    i = 0
    for surface in surfaces:
        glColor3f(colors[i][0],colors[i][1],colors[i][2])
        i += 1
        for vertex in surface:
            verts = vertices[vertex] 
            #print(vertices[vertex])
            glVertex3f(verts[0],verts[1],verts[2])

    
    glEnd()
    glPopMatrix()

    max_val = 0.5
    h = 0.07
    oneFace(2.3,-3.3,max_val,h,roll,1,"ROLL")
    oneFace(3.4,-3.3,max_val,h,pitch,4, "PITCH")
    oneFace(4.5,-3.3,max_val,h,yaw,3, "YAW")

    max_val= 0.05
    barVer(2.5,2.5,h,max_val,motors[0],2, "Motor LF")
    barVer(4,2.5,h,max_val,motors[1],2, "Motor RF")
    barVer(2.5,1,h,max_val,motors[2],2, "Motor LB")
    barVer(4,1,h,max_val,motors[3],2, "Motor RB")

    barHor(-3,-2,1,h,control[3],2, "Throt")
    barHor(-3,-2.4,1,h,control[0],2, "Roll")
    barHor(-3,-2.8,1,h,control[1],2, "Pitch")
    barHor(-3,-3.2,1,h,control[2],2, "Yaw")




def oneFace(pos_x,pos_y,max_val,h,roll, color, text):
    glPushMatrix()
    glTranslatef(pos_x, pos_y, 0)
    drawText((0, .6, 0), text, 14)
    drawText((0, .4, 0), "{:.2f}".format(roll)+"°", 12)
    glRotatef(-roll, 0.00, 0.00, 1.00)
    glBegin(GL_QUADS)
    
    vertices = [
        (max_val , -h, -max_val),
        (max_val , h, -max_val),
        (-max_val , h, -max_val),
        (-max_val , -h, -max_val),
        (max_val , -h, max_val),
        (max_val , h, max_val),
        (-max_val , -h, max_val),
        (-max_val , h, max_val),

    ]

    surfaces = [
        (6,7,5,4),

    ]

    colors = [
        (0,1,0),
        (1,0.5,0),
        (1,0,0),
        (1,1,0),
        (0,0,1),
        (1,0,1),
    ]
    
    i = 0
    for surface in surfaces:
        glColor3f(colors[color][0],colors[color][1],colors[color][2])
        i += 1
        for vertex in surface:
            verts = vertices[vertex] 
            #print(vertices[vertex])
            glVertex3f(verts[0],verts[1],verts[2])

    
    glEnd()
    glPopMatrix()

def barVer(pos_x,pos_y,max_val,h1,roll, color, text):
    if(roll==0):
        roll = 900
    realval = h1 + (roll-900)/2000
    h = realval
    glPushMatrix()
    glTranslatef(pos_x, pos_y, 0)
    drawText((0.2, .6, 0), text, 14)
    drawText((0.2, 0, 0), str(roll), 12)
    glBegin(GL_QUADS)
    
    vertices = [
        (max_val , 0, -max_val),
        (max_val , 2*h, -max_val),
        (-max_val , 2*h, -max_val),
        (-max_val , 0, -max_val),
        (max_val , 0, max_val),
        (max_val , 2*h, max_val),
        (-max_val , 0 , max_val),
        (-max_val , 2*h, max_val),

    ]

    surfaces = [
        (6,7,5,4),

    ]

    colors = [
        (0,1,0),
        (1,0.5,0),
        (1,0,0),
        (1,1,0),
        (0,0,1),
        (1,0,1),
    ]
    
    i = 0
    for surface in surfaces:
        glColor3f(colors[color][0],colors[color][1],colors[color][2])
        i += 1
        for vertex in surface:
            verts = vertices[vertex] 
            #print(vertices[vertex])
            glVertex3f(verts[0],verts[1],verts[2])

    
    glEnd()
    glPopMatrix()


def barHor(pos_x,pos_y,max_val1,h,roll, color, text):
    if(roll==0):
        roll = 900
    realval = max_val1 + (roll-1500)/1000
    max_val = realval
    glPushMatrix()
    glTranslatef(pos_x, pos_y, 0)
    drawText((0, 0.1, 0), text, 14)
    drawText((1.5, 0.1, 0), str(roll), 12)
    glBegin(GL_QUADS)
    
    vertices = [
        (0 , -h, max_val),
        (0 , h, max_val),
        (max_val , h, max_val),
        (max_val , -h, max_val),
        (0 , -h, 0),
        (0 , h, 0),
        (max_val , -h, 0),
        (max_val , h, 0),

    ]

    surfaces = [
        (6,7,5,4),

    ]

    colors = [
        (0,1,0),
        (1,0.5,0),
        (1,0,0),
        (1,1,0),
        (0,0,1),
        (1,0,1),
    ]
    
    i = 0
    for surface in surfaces:
        glColor3f(colors[color][0],colors[color][1],colors[color][2])
        i += 1
        for vertex in surface:
            verts = vertices[vertex] 
            #print(vertices[vertex])
            glVertex3f(verts[0],verts[1],verts[2])

    
    glEnd()
    glPopMatrix()



def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

#Primer método en ser llamado por default
if __name__ == '__main__':
    try:
	    init()
    except rospy.ROSInterruptException:
        print("Chaooo")
pass
