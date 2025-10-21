#!/usr/bin/env python3
# encoding: utf-8
import os
import re
import cv2
import sys
import math
import time
import rospy
import addcolor
import threading
import numpy as np
from Ui import Ui_Form
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from sensor_msgs.msg import Image

class MainWindow(QWidget, Ui_Form):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)

        self.color = 'red'
        self.L_Min = 0
        self.A_Min = 0
        self.B_Min = 0
        self.L_Max = 255
        self.A_Max = 255
        self.B_Max = 255
        self.kernel_erode = 3
        self.kernel_dilate = 3
        self.camera_opened = False
        
        self.horizontalSlider_LMin.valueChanged.connect(lambda: self.horizontalSlider_labvaluechange('lmin'))
        self.horizontalSlider_AMin.valueChanged.connect(lambda: self.horizontalSlider_labvaluechange('amin'))
        self.horizontalSlider_BMin.valueChanged.connect(lambda: self.horizontalSlider_labvaluechange('bmin'))
        self.horizontalSlider_LMax.valueChanged.connect(lambda: self.horizontalSlider_labvaluechange('lmax'))
        self.horizontalSlider_AMax.valueChanged.connect(lambda: self.horizontalSlider_labvaluechange('amax'))
        self.horizontalSlider_BMax.valueChanged.connect(lambda: self.horizontalSlider_labvaluechange('bmax'))

if __name__ == "__main__":
    rospy.init_node('lab_config')
    app = QApplication(sys.argv)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())
