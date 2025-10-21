#!/usr/bin/env python3
# encoding:utf-8
import sys
import time
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from arm_kinematics.InverseKinematics import *

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import  setServoPulse

#机械臂根据逆运动学算出的角度进行移动(the robotic arm moves according to the angles calculated by inverse kinematics)
ik = IK()

class ArmIK:
    
    servo10Range = (500, 2500, 0, 180)
    servo11Range = (500, 2500, 0, 180)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo10_Range=servo10Range, servo11_Range=servo11Range):
        # 适配不同的舵机(adapt to different servos)
        
        self.servo10Range = servo10_Range
        self.servo11Range = servo11_Range
       
    
    def transformAngelAdaptArm(self, theta, beta):
        #将逆运动学算出的角度转换为舵机对应的脉宽值(convert the angles calculated by inverse kinematics to the corresponding PWM values for the servos)
        servo11 = int(round(self.servo11Range[0]+((theta - self.servo11Range[2])/(self.servo11Range[3]-self.servo11Range[2]))*(self.servo11Range[1]-self.servo11Range[0])))

        if servo11 > self.servo11Range[1]  or servo11 < self.servo11Range[0]: #限位最大只转到2400，防止顶到支架(limit the maximum rotation to 2400 to prevent hitting the bracket)
            print('servo11的位置%s超出范围(%s, %s)'%(servo11, self.servo11Range[0], self.servo11Range[1]))
            return False

        servo10 = int(round(self.servo10Range[0]+((beta - self.servo10Range[2])/(self.servo10Range[3]-self.servo10Range[2]))*(self.servo10Range[1]-self.servo10Range[0])))
        if servo10 > self.servo10Range[1]  or servo10 < self.servo10Range[0]:#限位最大只转动1900(limit the maximum rotation to 1900)
            print('servo10的位置%s超出范围(%s, %s)'%(servo10, self.servo10Range[0], self.servo10Range[1]))
            return False

        return {"servo10": servo10, "servo11": servo11}

    def servosMove(self, servos, movetime=None):
        
        time.sleep(0.02)
        if movetime is None:
            max_d = 0
            movetime = int(max_d*1)
        setServoPulse(10, servos[0], movetime)
        setServoPulse(11, servos[1], movetime)

        return movetime
    
    def setPitchRange(self,coordinate_data):
        x, y, z = coordinate_data
        result = ik.getRotationAngle(coordinate_data)
        if result != False:
            theta, beta = result['theta'], result['beta']
            servos = self.transformAngelAdaptArm(theta, beta)
            if servos != False:
                return servos
            else:
                return False
        else:
            return False
            
    def setPitchRangeMoving(self, coordinate_data, movetime = None):
        #给定坐标coordinate_data计算出各个舵机的角度并转到目标位置(given the coordinate data, calculate the angles for each servo and move them to the target position)
        #如果无解返回False,否则返回舵机角度、运行时间(if no solution is found, return False; otherwise, return the servo angles and runtime)
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)(the coordinates are in centimeters and are passed in as a tuple, for example, (0, 5, 10))
        #movetime为舵机转动时间，单位ms, (movetime is the duration for the servo to rotate, measured in milliseconds)
        x, y, z = coordinate_data
        servos = self.setPitchRange(coordinate_data)
        #print(servos)
        if servos != False:
            movetime = self.servosMove((servos["servo10"], servos["servo11"]), movetime)  
            
            return servos, movetime
        else:
            return False
        
        
        
 
if __name__ == "__main__":
    AK = ArmIK()
    
    AK.setPitchRangeMoving((8.3,0,4),500)
    time.sleep(0.5)
    # for i in range(12,-6,-1):
    #     AK.setPitchRangeMoving((12,0,i),10)
    #     #AK.setPitchRange((10,0,i))
    #     time.sleep(0.01)
    # print(AK.setPitchRangeMoving((13,0,1),300))
    # time.sleep(2)
    # print(AK.setPitchRangeMoving((-4.8, 15, 1.5), 0, -90, 0, 2000))
    # AK.drawMoveRange2D(-10, 10, 0.2, 10, 30, 0.2, 2.5, -90, 90, 1)
