#!/usr/bin/env python3
# encoding: utf-8
# 2自由度机械臂逆运动学：给定相应的坐标（X,Y,Z），计算出每个关节转动的角度(inverse kinematics for a 2-degree-of-freedom robotic arm: Given the corresponding coordinates (X, Y, Z), calculate the angle of rotation for each joint)

from math import *


class IK:        #从下往上，最下面为第一个舵机(from bottom to top, the bottommost servo is considered the first servo)
    l1 = 4.21    #机械臂第一个舵机中心轴到第二个舵机中心轴的距离4.21cm,(the distance from the center axis of the first servo to the center axis of the second servo on the robotic arm is 4.21 cm)
    l2 = 3.3     #第二个舵机中心轴到爪子末端所在平面的距离3.3cm(the distance from the center axis of the second servo to the plane where the end of the gripper is located is 3.3 cm)
    l3 = 12.7    #第二个舵机中心轴垂直于爪子末端所在平面的交点到爪子末端的距离12.7cm(the distance from the intersection of the center axis of the second servo perpendicular to the plane where the end of the gripper is located to the end of the gripper is 12.7 cm)
    
    def __init__(self): # 舵机从下往上数(count the servos from bottom to top)
        l1 = 4.21    #机械臂第一个舵机中心轴到第二个舵机中心轴的距离4.21cm,(count the servos from bottom to top)
        l2 = 3.3     #第二个舵机中心轴到爪子末端所在平面的距离3.3cm(the distance from the center axis of the second servo to the plane where the end of the gripper is located is 3.3 cm)
        l3 = 12.7    #第二个舵机中心轴垂直于爪子末端所在平面的交点到爪子末端的距离12.7cm(the distance from the intersection of the center axis of the second servo perpendicular to the plane where the end of the gripper is located to the end of the gripper is 12.7 cm)

        

    def setLinkLength(self, L1=l1, L2=l2, L3=l3):
        # 更改机械臂的连杆长度，为了适配相同结构不同长度的机械臂(change the linkage length of the robotic arm to adapt to arms of the same structure but different lengths)
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3

    def getLinkLength(self):
        # 获取当前设置的连杆长度(get the current set linkage length)
        
        return {"L1":self.l1, "L2":self.l2, "L3":self.l3}

    def getRotationAngle(self, coordinate_data):
        
        X, Y, Z = coordinate_data
        # 没有左右转向，Y都为0(without left or right turning, Y is always 0)
        
        if Y != 0:
            print("ERROR:给的坐标Y值必须为0")
            return False
        
        # 给定指定坐标，返回每个关节应该旋转的角度，如果无解返回False(given specific coordinates, return the angle each joint should rotate to. If there's no solution, return False)
        # coordinate_data为夹持器末端坐标，坐标单位cm， 以元组形式传入，例如(5, 0, 10)(the coordinate_data represents the end effector coordinates of the gripper, with units in centimeters, passed in as a tuple, for example, (5, 0, 10))
        
        # 设夹持器末端为P(X, Y, Z), 坐标原点为O, 原点为机械臂第一个舵机中心轴(let the end effector of the gripper be denoted as P(X, Y, Z), with the origin at O, where O is the center axis of the first servo of the robotic arm)
        # 从机械臂后面往前看，正前方为X轴正方向，Z轴竖直向上为正方向，遵循右手定则，Y轴往左边为正方向(viewing the robotic arm from behind, the positive X-axis direction is straight ahead, the positive Z-axis direction is vertically upwards, following the right-hand rule, and the positive Y-axis direction is to the left)
        # l1与l2的交点为A(the intersection point of l1 and l2 is denoted as point A)
        # 夹角表示，AB和BC的夹角为 ABC(the angle formed by AB and BC is denoted as ABC)
        # l1与X轴的夹角为theta(the angle between l1 and the X-axis is denoted as theta)
        # l1与l2的夹角为OAP(the angle between l1 and l2 is denoted as OAP)
        # theta为第一个舵机转动角度(theta is the rotation angle of the first servo)
        # beta为第二个舵机转动角度(beta is the rotation angle of the second servo)
        #
        
        
        PO = sqrt(X*X + Z*Z) #P到原点O距离，Y为0所以省略不写(the distance from point P to the origin O is denoted, with Y being 0, which is omitted)
        PA = sqrt(pow(self.l2,2) + pow(self.l3,2)) #因为l2与l3为垂直关系，根据勾股定理可以算出斜边PA(since l2 and l3 are perpendicular, according to the Pythagorean theorem, we can calculate the hypotenuse PA)

        #print("PO:",PO)
        #Z不同的范围，theta的计算方式不一样，有三种情况，需要分类讨论，beta三种情况里计算方式都一样(depending on the range of Z, there are three cases for calculating theta, and beta is the same in all three cases)
        #beta = OAP

        if X == -3.3 and Z == (self.l1 + self.l3 ):
            theta = 90
            beta = 180 
            return {"theta":theta, "beta":beta} # 有解时返回角度字典(return a dictionary of angles when a solution exists)
        
        if X == (self.l1 + self.l3 ) and Z == 3.3:
            theta = 180
            beta = 180 
            return {"theta":theta, "beta":beta} # 有解时返回角度字典(return a dictionary of angles when a solution exists)
        
        cos_OAP = (pow(self.l1,2)+pow(PA,2)-pow(PO,2))/(2*self.l1*PA)#根据余弦定理(according to the law of cosines)
        cos_OAP = round(cos_OAP,4) #取四位小数点(take four decimal places)
        if abs(cos_OAP) > 1:
            print('ERROR:不能构成连杆结构, abs(cos_OAP)为%s > 1' %(abs(cos_OAP)) )
            return False
        
        cos_PAC = (pow(self.l2,2)+pow(PA,2)-pow(self.l3,2))/(2*self.l2*PA)#根据余弦定理(according to the law of cosines)
        cos_PAC = round(cos_PAC,4) #取四位小数点(take four decimal places)
        if abs(cos_PAC) > 1:
            print('ERROR:不能构成连杆结构, abs(cos_PAC)为%s > 1' %(abs(cos_PAC)))
            return False
        
        OAP = acos(cos_OAP)
        PAC = acos(cos_PAC)
        OAC = OAP + PAC
        beta = degrees(OAC) - 90
        
        #当Z>0时，PB垂直于OB，B点在X轴上，AC垂直于X轴，C在X轴上(when Z > 0, PB is perpendicular to OB, point B is on the X-axis, and AC is perpendicular to the X-axis, with point C on the X-axis)
        # theta = AOB
        # AOB = AOP + POB
       
        if Z > 0:
            
            PB = Z
            
            cos_AOP = (pow(self.l1,2)+pow(PO,2)-pow(PA,2))/(2*self.l1*PO)#根据余弦定理(according to the law of cosines)
            cos_AOP = round(cos_AOP,4) #取四位小数点(take four decimal places)
            if abs(cos_AOP) > 1:
               print('ERROR:不能构成连杆结构, abs(cos_AOP)为%s > 1' %(abs(cos_AOP)))
               return False
        
            sin_POB = round(PB/PO,4)
            if abs(sin_POB) > 1:
               print('ERROR:不能构成连杆结构, abs(sin_POB)为%s > 1' %(abs(sin_POB)) )
               return False
           
            AOB = acos(cos_AOP) + asin(sin_POB) #l1与x轴的夹角(the angle between l1 and the X-axis)
           
            theta = 180 - degrees(AOB)    
            
        #当Z为0时，P点在X轴上，theta = AOP(when Z is 0, point P is on the X-axis, and theta equals AOP)
        elif Z == 0:
            
            cos_AOP = (pow(self.l1,2)+pow(PO,2)-pow(PA,2))/(2*self.l1*PO)#根据余弦定理(according to the law of cosines)
            cos_AOP = round(cos_AOP,4) #取四位小数点(take four decimal places)
            
            if abs(cos_AOP) > 1:
               print('ERROR:不能构成连杆结构, abs(cos_AOP)为%s > 1' %(abs(cos_AOP)))
               return False
            
           
            AOB = acos(cos_AOP)
            theta = 180 - degrees(AOB)
            
        #当Z<0时，PB垂直于OB，B在X轴上，theta = AOP - AOB(when Z is less than 0, PB is perpendicular to OB, point B is on the X-axis, and theta equals AOP minus AOB)
        elif Z < 0:
            PB = abs(Z) 

            
            cos_AOP = (pow(self.l1,2)+pow(PO,2)-pow(PA,2))/(2*self.l1*PO)#根据余弦定理(according to the law of cosines)
            cos_AOP = round(cos_AOP,4) #取四位小数点(take four decimal places)
            if abs(cos_AOP) > 1:
               print('ERROR:不能构成连杆结构, abs(cos_AOP)为%s > 1' %(abs(cos_AOP)))
               return False
            sin_POB = round(PB/PO,4)
            if abs(sin_POB) > 1:
               print('ERROR:不能构成连杆结构, abs(sin_POB)为%s > 1' %(abs(sin_POB)) )
               return False
           
            AOB = acos(cos_AOP) - asin(sin_POB) #l1与x轴的夹角(the angle between l1 and the X-axis)
            theta = 180 - degrees(AOB)    
 
 
        if self.l1 + PA < round(PO, 4): #两边之和小于第三边(the sum of any two sides is always greater than the third side)
            print('ERROR:不能构成连杆结构, l1(%s) + PA(%s) < PO(%s)' %(self.l1, PA, PO))
            return False
        
        return {"theta":theta, "beta":beta} # 有解时返回角度字典(return a dictionary of angles when a solution exists)
            
if __name__ == '__main__':
    ik = IK()
    #ik.setLinkLength(L1=ik.l1 + 1.30, L4=ik.l4)
    #print('连杆长度：', ik.getLinkLength())
    print(ik.getRotationAngle((5, 0, 0)))
