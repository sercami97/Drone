#!/usr/bin/env python3
# coding=utf-8
import sys
import rospy
# from pynput.keyboard import Key, Listener
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray, String, Int32

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.uic import *

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

class mainWindow(QMainWindow):

    verticies = (
            (1, -1, -1),
            (1, 1, -1),
            (-1, 1, -1),
            (-1, -1, -1),
            (1, -1, 2),
            (1, 1, 1),
            (-1, -1, 1),
            (-1, 1, 1)
        )

    edges = (
             (0,1),
             (0,3),
             (0,4),
             (2,1),
             (2,3),
             (2,7),
             (6,3),
             (6,4),
             (6,7),
             (5,1),
             (5,4),
             (5,7)
            )

    colors = (
              (1,0,0),
              (0,1,0),
              (0,0,1),
              (0,1,0),
              (1,1,1),
              (0,1,1),
              (1,0,0),
              (0,1,0),
              (0,0,1),
              (1,0,0),
              (1,1,1),
              (0,1,1),
             )

    surfaces = (
                (0,1,2,3),
                (3,2,7,6),
                (6,7,5,4),
                (4,5,1,0),
                (1,5,7,2),
                (4,0,3,6)
               )

    def __init__(self):
        super(mainWindow, self).__init__()
        self.width = 700    #Variables used for the setting of the size of everything
        self.height = 600
        self.setGeometry(0, 0, self.width, self.height)    #Set the window size

    def setupUI(self):
        self.openGLWidget = QOpenGLWidget(self)    #Create the GLWidget
        self.openGLWidget.setGeometry(0, 0, self.width, self.height)    #Size it the same as the window.
        self.openGLWidget.initializeGL()
        self.openGLWidget.resizeGL(self.width, self.height)    #Resize GL's knowledge of the window to match the physical size?
        self.openGLWidget.paintGL = self.paintGL    #override the default function with my own?
        self.openGLWidget.resizeGL = self.resizeGL

    def resizeGL(self, width, height):
        self.width, self.height = width, height
        # update viewport
        glViewport(0, 0, self.width, self.height)
        # set projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, self.width / self.height, 0.1, 50.0)    #set perspective?


    def paintGL(self):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0, 0, -10)    #I used -10 instead of -2 in the PyGame version.
        glRotatef(-90, 1, 0, 0)    #I used 2 instead of 1 in the PyGame version.

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)    #Straight from the PyGame version, with 'self' inserted occasionally

        glBegin(GL_QUADS)    #tell GL to draw surfaces
        for surface in self.surfaces:
            x = 0
            for vertex in surface:
                x+=1
                glColor3fv(self.colors[x])
                glVertex3fv(self.verticies[vertex])
        glEnd()    #tell GL to stop drawing surfaces



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
    
    
    try:
        rate = rospy.Rate(10)
        print("Velocidad")
        m.data = [0,0]
        app = QApplication([])
        window = mainWindow()
        window.setupUI()
        

        while not rospy.is_shutdown():
            print("Running")
            window.show()
            rate.sleep()
        #sys.exit(app.exec_())

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
