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

from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

useQuat = False   # set true for using quaternions, false for using y,p,r angles
gyr = [0.0,0.0,0.0]
voltage = 0

m = Float32MultiArray()

def callBackSensores(msg):
    global gyr, voltage
    print("Callback ", msg.data)
    
    gyr[0] = msg.data[0]
    gyr[1] = (-1)*msg.data[1]
    gyr[2] = (-1)*msg.data[2]

    voltage = msg.data[7]/10
    print("Voltage: ", voltage)

#Método constructor. Se establecen las conexion y se inicializan variables.
def init():
    global frames
    rospy.init_node('SensoresRecibir', anonymous=True)
    rospy.Subscriber('sensores', Float32MultiArray, callBackSensores)#Control
    initGame()
    
    
    try:
        rate = rospy.Rate(10)
        print("Velocidad")
        m.data = [0,0]

        while not rospy.is_shutdown():
            event = pygame.event.poll()
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                break
            if(useQuat):
                [w, nx, ny, nz] = read_data()
            else:
                [yaw, pitch, roll] = read_data()
            if(useQuat):
                draw(w, nx, ny, nz)
            else:
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
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
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
                
    if(useQuat):
        w = float(1.0)
        nx = float(0.0)
        ny = float(1.0)
        nz = float(0.0)
        return [w, nx, ny, nz]
    else:
        yaw = float(gyr[2])
        pitch = float(gyr[1])
        roll = float(gyr[0])
        return [yaw, pitch, roll]


def draw(w, nx, ny, nz):
    global voltage
    estado = "Armado"
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0.0, 0.0, -10.0)

    drawText((0, 1.8, 2), "PyTeapot", 20)
    drawText((-2.6, 1.6, 2), "Estado: %s, Voltaje: %.2f" %(estado,voltage), 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)
    drawText((-2.6, -1.6, 2), "Press Escape to exit.", 16)


    if(useQuat):
        [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
        drawText((-1.6, -1, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)
    else:
        yaw = nx
        pitch = ny
        roll = nz
        drawText((-1.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)

    glPushMatrix()
    glTranslatef(-2.5, 0.0, 0)
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
            print(vertices[vertex])
            glVertex3f(verts[0],verts[1],verts[2])

    
    glEnd()
    glPopMatrix()

    glPushMatrix()
    glTranslatef(2.5, 0.0, 0)
    glRotatef(-roll, 0.00, 0.00, 1.00)
    # glRotatef(0,0, 0.00, 0.00)

    glBegin(GL_QUADS)
    
    max_val = 1
    h = 0.1
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
        glColor3f(colors[i][0],colors[i][1],colors[i][2])
        i += 1
        for vertex in surface:
            verts = vertices[vertex] 
            print(vertices[vertex])
            glVertex3f(verts[0],verts[1],verts[2])

    
    glEnd()
    glPopMatrix()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.sin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]


#Primer método en ser llamado por default
if __name__ == '__main__':
    try:
	    init()
    except rospy.ROSInterruptException:
        print("Chaooo")
pass
