#!/usr/bin/python3
# coding=utf8
# Author:Summer
# Email:997950600@qq.com

import os, sys, math
import numpy as np
import rospy
from std_msgs.msg import UInt8, UInt16, Float32, Float64, Bool, String,Float32MultiArray
from std_srvs.srv import Empty, SetBool
# from sensor_msgs.msg import Image
from ros_robot_controller.msg import BuzzerState
from geometry_msgs.msg import Point32, Polygon
from puppy_control.msg import Velocity, Pose, Gait,SetServo
from puppy_control.srv import SetRunActionName
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from arm_kinematics.ArmMoveIK import *

ROS_NODE_NAME = 'puppy_control'

#from HiwonderPuppy import HiwonderPuppy, PWMServoParams
sys.path.append('/home/ubuntu/software/puppypi_control')
from servo_controller import setServoPulse, updatePulse
from action_group_control import runActionGroup, stopActionGroup


from HiwonderPuppy import HiwonderPuppy, PWMServoParams

with_arm = 0
if with_arm:
    offset = 0.1
else:
    offset = 0

Stand = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5+offset, 'stance_x':0, 'stance_y':0}
#正常4腿站立
LieDown = {'roll':0.000, 'pitch':0.000, 'yaw':0.000, 'height':-5, 'x_shift':2, 'stance_x':0, 'stance_y':0}
#趴下
LookDown = {'roll':math.radians(0), 'pitch':math.radians(-15), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
LookDown_10deg = {'roll':math.radians(0), 'pitch':math.radians(-10), 'yaw':0.000, 'height':-9, 'x_shift':-0.1, 'stance_x':0, 'stance_y':0}
#俯身 10度
LookDown_20deg = {'roll':math.radians(0), 'pitch':math.radians(-20), 'yaw':0.000, 'height':-9, 'x_shift':-0.1, 'stance_x':0, 'stance_y':0}
#向下看
LookDown_30deg = {'roll':math.radians(0), 'pitch':math.radians(-30), 'yaw':0.000, 'height':-9.6, 'x_shift':-1.4, 'stance_x':1, 'stance_y':0}

StandLow = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-7, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
#4腿站立的低姿态
PuppyPose = LieDown.copy()
# stance_x：4条腿在x轴上额外分开的距离，单位cm
# stance_y：4条腿在y轴上额外分开的距离，单位cm
# x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm
# height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm
# pitch： 狗身体的俯仰角，单位弧度


GaitConfigFast = {'overlap_time':0.1, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':5}
# 快速

GaitConfigSlow = {'overlap_time':0.4, 'swing_time':0.3, 'clearance_time':0.26, 'z_clearance':4}
# 慢速
GaitConfigMarkTime = {'overlap_time':0.2, 'swing_time':0.1, 'clearance_time':0.0, 'z_clearance':5}
# GaitConfig = {'overlap_time':0.1, 'swing_time':0.1, 'clearance_time':0.0, 'z_clearance':4}
# GaitConfigMarkTime = {'overlap_time':0.2, 'swing_time':0.1, 'clearance_time':0.0, 'z_clearance':4}
# GaitConfigCrawl = {'overlap_time':0.4, 'swing_time':0.3, 'clearance_time':0.20, 'z_clearance':4}
GaitConfig = GaitConfigFast.copy()


# overlap_time:4脚全部着地的时间，单位秒
# swing_time：单脚离地时间，单位秒
# clearance_time：前后交叉脚相位间隔时间，单位秒
# z_clearance：走路时，脚尖要抬高的距离，单位cm

class MPU6050():

    def __init__(self):
        rospy.Subscriber('/ros_robot_controller/imu_raw', Imu, self.GetImuFun)
        self.data = {'accel':[0,0,0], 'gyro':[0,0,0]}
        self.SecondOrderFilterX = self._SecondOrderFilter()
        self.SecondOrderFilterY = self._SecondOrderFilter()
        
    def GetImuFun(self,msg):
        self.data['accel'][0] = msg.linear_acceleration.x
        self.data['accel'][1] = msg.linear_acceleration.y
        self.data['accel'][2] = msg.linear_acceleration.z
        self.data['gyro'][0] = msg.angular_velocity.x
        self.data['gyro'][1] = msg.angular_velocity.y
        self.data['gyro'][2] = msg.angular_velocity.z


    def _SecondOrderFilter(self):
        x1=0
        x2=0
        y1=0
        angle = 0
        K2 =0.02
        def fun( angle_m, gyro_m, dt = 0.01):
            nonlocal x1
            nonlocal x2
            nonlocal y1
            nonlocal angle
            nonlocal K2
            x1=(angle_m-angle)*(1-K2)*(1-K2)
            y1=y1+x1*dt
            x2=y1+2*(1-K2)*(angle_m-angle)+gyro_m
            angle=angle+ x2*dt
            return angle

        return fun

    def get_euler_angle(self, dt = 0.01):
        # data = self.get_all_data()
        data = self.data
        accel_Y = math.atan2(data['accel'][0],data['accel'][2])*180/math.pi
        gyro_Y = data['gyro'][1]
        angleY = self.SecondOrderFilterY(-accel_Y,gyro_Y,dt)

        accel_X = math.atan2(data['accel'][1],data['accel'][2])*180/math.pi
        gyro_X = data['gyro'][0]
        angleX = self.SecondOrderFilterX(accel_X,gyro_X,dt)

        # return {'pitch':0, 'roll':0, 'yaw':0}
        return {'pitch':-math.radians(angleX), 'roll':-math.radians(angleY), 'yaw':0}


class PUPPY():
    def __init__(self):
        
        rospy.set_param('/%s/PuppyPose' %ROS_NODE_NAME,
            {'Stand':Stand, 'LieDown':LieDown, 'LookDown':LookDown, 'LookDown_10deg':LookDown_10deg, 
            'LookDown_20deg':LookDown_20deg, 'LookDown_30deg':LookDown_30deg, 'StandLow':StandLow})
        rospy.set_param('/%s/GaitConfig' %ROS_NODE_NAME,{'GaitConfigFast':GaitConfigFast, 'GaitConfigSlow':GaitConfigSlow})
        rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)#DEBUG INFO
        self.joint_state_pub_topic = rospy.get_param('~joint_state_pub_topic' ,'false')
        self.joint_state_controller_pub_topic = rospy.get_param('~joint_state_controller_pub_topic' ,'false')
        self.puppy = HiwonderPuppy(setServoPulse = setServoPulse, servoParams = PWMServoParams(),dof='8')
        self.mpu = MPU6050()
        self.puppy.imu = None#self.mpu
        

        self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        # puppy.gait_config(overlap_time = 0.1, swing_time = 0.15, z_clearance = 3)
        self.puppy.gait_config(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                                , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
        # overlap_time:4脚全部着地的时间，swing_time：2脚离地时间，z_clearance：走路时，脚抬高的距离
   
        self.puppy.start() # 启动
        self.puppy.move_stop(servo_run_time = 500)#
        self.ak = ArmIK()
        self.ak.setPitchRangeMoving((8.51,0,3.3),500)
        rospy.sleep(0.5)
        setServoPulse(9,1500,300)
        rospy.sleep(0.3)
        # self.puppy.move(x=0, y=0, yaw_rate = 0.0)#18/57.3


        rospy.Service('/%s/set_running' %ROS_NODE_NAME, SetBool, self.set_running)
        rospy.Service('/%s/go_home' %ROS_NODE_NAME, Empty, self.go_home)
        rospy.Service('/%s/set_self_balancing' %ROS_NODE_NAME, SetBool, self.set_self_balancing)
        rospy.Service('/%s/runActionGroup' %ROS_NODE_NAME, SetRunActionName, self.runActionGroupFun)
        rospy.Service('/%s/set_mark_time' %ROS_NODE_NAME, SetBool, self.set_mark_time)


        rospy.Subscriber('/%s/gait' %ROS_NODE_NAME, Gait, self.GaitFun)
        rospy.Subscriber('/%s/velocity' %ROS_NODE_NAME, Velocity, self.VelocityFun)
        rospy.Subscriber('/%s/velocity_move' %ROS_NODE_NAME, Velocity, self.VelocityMoveFun)
        rospy.Subscriber('/%s/velocity/autogait' %ROS_NODE_NAME, Velocity, self.VelocityAutogaitFun)
        rospy.Subscriber('/cmd_vel', Twist, self.Cmd_velFun)
        rospy.Subscriber('/cmd_vel_nav', Twist, self.Cmd_vel_nav_Fun)
        rospy.Subscriber('/%s/pose' %ROS_NODE_NAME, Pose, self.PoseFun)
        rospy.Subscriber('/%s/fourLegsRelativeCoordControl' %ROS_NODE_NAME, Polygon, self.fourLegsRelativeCoordControlFun)
        rospy.Subscriber('/%s/gait/pc' %ROS_NODE_NAME, Float32MultiArray, self.GaitPCFun)
        rospy.Subscriber('/%s/setServo' %ROS_NODE_NAME, SetServo, self.SetServo_Fun)

        self.legs_coord_pub = rospy.Publisher('/%s/legs_coord' %ROS_NODE_NAME, Polygon, queue_size=1)
        # self.legs_servo_value_pub = rospy.Publisher('/%s/legs_servo_value' %ROS_NODE_NAME, Float32MultiArray, queue_size=1)

        self.joint_state = JointState()
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.joint_state.name = ['rf_joint1', 'lf_joint1', 'rb_joint1', 'lb_joint1', 'rf_joint2', 'lf_joint2', 'rb_joint2', 'lb_joint2']
        command_topics = ["/puppy/joint1_position_controller/command",
                          "/puppy/joint2_position_controller/command",
                          "/puppy/joint3_position_controller/command",
                          "/puppy/joint4_position_controller/command",
                          "/puppy/joint5_position_controller/command",
                          "/puppy/joint6_position_controller/command",
                          "/puppy/joint7_position_controller/command",
                          "/puppy/joint8_position_controller/command"]

        self.joint_controller_publishers = []
        for i in range(len(command_topics)):
            self.joint_controller_publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size=10))


        self.rate = rospy.Rate(100)  # 100hz
        
        buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        rospy.sleep(0.2) # pub之后必须延时才能生效
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.1
        msg.off_time = 0.9
        msg.repeat = 1
        buzzer_pub.publish(msg)


    def stance(self, x = 0, y = 0, z = -11, x_shift = 2):# 单位cm
        # x_shift越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡
        return np.array([
                            [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
                            [y, y, y, y],
                            [z, z, z, z],
                        ])#此array的组合方式不要去改变
    
    def SetServo_Fun(self, msg):
        
        if msg.pulse > 2500:msg.pulse = 2500
        if msg.pulse < 500: msg.pulse = 500
        if msg.time > 30000:msg.time = 30000
        print(f"Set Servo ID: {msg.id}, Pulse: {pulse}, Time: {time}")
        rospy.loginfo(f"Set Servo ID: {msg.id}, Pulse: {pulse}, Time: {time}")
        
        setServoPulse(msg.id,msg.pulse,msg.time)

    def GaitFun(self, msg):
        global GaitConfig
        rospy.logdebug(msg)
        # GaitConfig = GaitConfigMarkTime,copy()
        GaitConfig = dict(zip(GaitConfig.keys(), msg.__getstate__()))
        self.puppy.gait_config(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                                , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    def GaitPCFun(self, msg):
        global GaitConfig
        # data:[params.gait, params.height, params.period, params.x, params.y, params.yaw]
        rospy.logdebug(msg)
        if msg.data[0] == 0:
            self.puppy.move_stop(servo_run_time = 100)
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        else:
            # 一个步态完整的周期 = 2*overlap_time + 2*swing_time + 2*clearance_time
            if msg.data[0] == 1: # Trot
                # clearance_time固定为0。这里设定overlap_time = swing_time
                GaitConfig['overlap_time'] = msg.data[2] /4
                GaitConfig['swing_time'] = msg.data[2] /4
                GaitConfig['clearance_time'] = 0
            elif msg.data[0] == 2: # Amble
                # 这里设定clearance_time为swing_time的一半，同时overlap_time = swing_time
                GaitConfig['overlap_time'] = msg.data[2] /5
                GaitConfig['swing_time'] = msg.data[2] /5
                GaitConfig['clearance_time'] = msg.data[2] /10
            elif msg.data[0] == 3: # Walk
                # 这里设定clearance_time = swing_time，同时overlap_time = swing_time
                GaitConfig['overlap_time'] = msg.data[2] /6
                GaitConfig['swing_time'] = msg.data[2] /6
                GaitConfig['clearance_time'] = msg.data[2] /6

            GaitConfig['z_clearance'] = msg.data[1]
            self.puppy.gait_config(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                            , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
            self.VelocityFun(Velocity(x=msg.data[3], y=msg.data[4], yaw_rate=msg.data[5]))
    
    def Cmd_velFun(self, msg):
        global PuppyPose
        rospy.logdebug(msg)

        if abs(msg.linear.x) > 0.5 or abs(msg.angular.z) > 0.5:
            PuppyPose = Stand.copy()
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])

            if abs(msg.linear.x) > abs(msg.angular.z):
                self.VelocityFun(Velocity(16*np.sign(msg.linear.x),0,0))
            else:
                self.VelocityFun(Velocity(0,0,np.radians(25)*np.sign(msg.angular.z)))
        elif msg.linear.x == 0 and msg.angular.z == 0:
            self.VelocityFun(Velocity(0,0,0))

    def Cmd_vel_nav_Fun(self, msg):
        global PuppyPose
        PuppyPose = Stand.copy()
        self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        self.VelocityFun(Velocity(msg.linear.x * 100,0,msg.angular.z))

    def VelocityMoveFun(self, msg):
        rospy.logdebug(msg)
        #rospy.loginfo(msg)
        if msg.x ==0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time = 100)
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
            
        else:
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate = msg.yaw_rate)

    def VelocityFun(self, msg):
        rospy.logdebug(msg)
        if msg.x == -999:# 原地踏步
            self.puppy.move(x=0, y=0, yaw_rate=0)
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        elif msg.x ==0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time = 100)
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        elif abs(msg.x) <= 35 and abs(msg.y) == 0 and abs(msg.yaw_rate) <= np.radians(51):
            if msg.x > 0:
                self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']-0.8), PuppyPose['pitch'], PuppyPose['roll'])
            else:
                self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']+0.8), PuppyPose['pitch'], PuppyPose['roll'])
            
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate = msg.yaw_rate)
            
    def VelocityAutogaitFun(self, msg):
        rospy.logdebug(msg)
        if msg.x ==0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time = 100)
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        elif abs(msg.x) <= 35 and abs(msg.y) == 0 and abs(msg.yaw_rate) <= np.radians(51):
            if abs(msg.x) <= 10:
                overlap_time_x = 0.45 - abs(msg.x) * 0.023
                swing_time_x = 0.38 - abs(msg.x) * 0.0154
                clearance_time_x = swing_time_x - 0.04
            elif abs(msg.x) <= 15:
                overlap_time_x = 0.45 - abs(msg.x) * 0.023
                swing_time_x = 0.38 - abs(msg.x) * 0.0154
                clearance_time_x = 0
            else:
                overlap_time_x = 0.1
                swing_time_x = 0.15
                clearance_time_x = 0
            #if msg.yaw_rate > 0: msg.yaw_rate = np.radians(20)
            #elif msg.yaw_rate < 0:msg.yaw_rate = np.radians(-20)
            if abs(msg.yaw_rate) <= np.radians(10):
                overlap_time_yaw_rate = 0.23 - abs(msg.yaw_rate) * 0.37
                swing_time_yaw_rate = 0.36 - abs(msg.yaw_rate) * 0.74
                clearance_time_yaw_rate = swing_time_yaw_rate - 0.04
            if abs(msg.yaw_rate) <= np.radians(20):
                overlap_time_yaw_rate = 0.23 - abs(msg.yaw_rate) * 0.37
                swing_time_yaw_rate = 0.41 - abs(msg.yaw_rate) * 0.74
                clearance_time_yaw_rate = 0
            else:
                overlap_time_yaw_rate = 0.1
                swing_time_yaw_rate = 0.15
                clearance_time_yaw_rate = 0

            GaitConfig['overlap_time'] = min(overlap_time_x, overlap_time_yaw_rate)
            GaitConfig['swing_time'] = min(swing_time_x, swing_time_yaw_rate)
            GaitConfig['clearance_time'] = min(clearance_time_x, clearance_time_yaw_rate)
            

            self.puppy.gait_config(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                                , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])

            if msg.x > 0:
                self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']-0.8), PuppyPose['pitch'], PuppyPose['roll'])
            else:
                self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']+0.8), PuppyPose['pitch'], PuppyPose['roll'])
            
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate = msg.yaw_rate)
            # self.puppy.move(x=7, y=msg.y, yaw_rate = msg.yaw_rate)

    def PoseFun(self, msg):
        global PuppyPose
        rospy.logdebug(msg)
        # return
        if (abs(msg.roll) <= np.radians(31) and abs(msg.pitch) <= np.radians(31) 
                and abs(msg.yaw) == 0 and msg.height >= -15 and msg.height <= -5
                and abs(msg.stance_x) <= 5 and abs(msg.stance_y) <= 5
                and abs(msg.x_shift) <= 10):

            if msg.run_time != 0:
                self.puppy.move_stop(servo_run_time = msg.run_time)
                rospy.sleep(0.01)
                self.puppy.servo_force_run()

            PuppyPose = dict(zip(PuppyPose.keys(), msg.__getstate__()[:-1]))
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])

    def fourLegsRelativeCoordControlFun(self, msg):
        rospy.logdebug(msg)
        rospy.logdebug(msg.points[0])

        rotated_foot_locations = np.zeros((3, 4))
        for idx, p in enumerate(msg.points):
            rotated_foot_locations[:, idx] = p.x,p.y,p.z
            
        joint_angles = self.puppy.fourLegsRelativeCoordControl(rotated_foot_locations)
        print(f"Calculated Joint Angles: {joint_angles}")
        self.puppy.sendServoAngle(joint_angles)
        

    def runActionGroupFun(self, msg):
        rospy.logdebug(msg)
        runActionGroup(msg.name, msg.wait)
        return [True, msg.name]

    def pub(self):
        times = 0
        while not rospy.is_shutdown():
            self.rate.sleep()

            coord = self.puppy.get_coord()

            if times >= 100:
                times = 0
                msg = Polygon(list(map(Point32, coord[0,:], coord[1,:], coord[2,:])))
                self.legs_coord_pub.publish(msg)
                rospy.sleep(0.002)
                self.joint_state_pub_topic = rospy.get_param('~joint_state_pub_topic' ,'false')
                self.joint_state_controller_pub_topic = rospy.get_param('~joint_state_controller_pub_topic' ,'false')
            
            times += 1

            # print('coord',coord)
            # print(msg)
            
            if self.joint_state_pub_topic or self.joint_state_controller_pub_topic:
                joint_angles = self.puppy.fourLegsRelativeCoordControl(coord/100)
                data = sum([list(joint_angles[1,:]),list(joint_angles[2,:])],[])
                # joint_angles *= 57.3
                self.joint_state.header.stamp = rospy.Time.now()
                for i in range(len(data)):
                    if i > 3:
                        data[i] = 0.0695044662*data[i]**3 - 0.0249173454*data[i]**2 - 0.786456081*data[i] + 1.5443387652 - 3.1415926/2 #-data[i]
                    if self.joint_state_controller_pub_topic:
                        self.joint_controller_publishers[i].publish(data[i])
                if self.joint_state_pub_topic:
                    self.joint_state.position = data 
                    self.joint_state_pub.publish(self.joint_state)
                
               
        print(ROS_NODE_NAME + " Shutting down")

    def set_running(self, msg):
        rospy.logdebug(msg)
        if msg.data:
            #updatePulse(-1)
            self.puppy.move_stop(servo_run_time = 500)
            self.puppy.start()
        else:
            self.puppy.move_stop(servo_run_time = 500)
            self.puppy.end()
        return [True, 'set_running']
        
    def set_self_balancing(self, msg):
        rospy.logdebug(msg)
        if msg.data:
            PuppyPose = Stand.copy()
            self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        
            self.puppy.move_stop(servo_run_time = 500)
            rospy.sleep(0.01)
            self.puppy.servo_force_run()
            rospy.sleep(0.5)
            self.puppy.move_stop(servo_run_time = 0)
            self.puppy.imu = self.mpu
        else:
            self.puppy.imu = None#self.mpu
        return [True, 'set_self_balancing']

    def set_mark_time(self, msg):
        rospy.logdebug(msg)
        if msg.data:
            self.go_home()
            # PuppyPose = Stand.copy()
            GaitConfig = GaitConfigMarkTime.copy()
            self.puppy.gait_config(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                                , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
            # self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']+0.8), PuppyPose['pitch'], PuppyPose['roll'])
            self.puppy.move(x=0, y=0, yaw_rate = 0)
            
        else:
            pass
        return [True, 'set_mark_time']

    def go_home(self, req=None):
        global PuppyPose
        rospy.logdebug('go_home')
        PuppyPose = Stand.copy()
        self.puppy.stance_config(self.stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        
        # updatePulse(-1)
        self.puppy.move_stop(servo_run_time = 500)
        rospy.sleep(0.01)
        self.puppy.servo_force_run()
        rospy.sleep(0.5)
        self.puppy.move_stop(servo_run_time = 0)
        return []


if __name__ == '__main__':
    puppy = PUPPY()

    try:
        puppy.pub()
    except :
        pass
    finally:
        pass
