#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/08/28
# stm32 ros2 package
import math
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import UInt16, Bool
from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller_msgs.srv import GetBusServoState, GetPWMServoState
from ros_robot_controller_msgs.msg import ButtonState, BuzzerState, LedState, MotorsState, BusServoState, SetBusServoState, ServosPosition, SetPWMServoState, Sbus, OLEDState, RGBsState

class RosRobotController(Node):
    gravity = 9.80665
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.board = Board()
        self.board.enable_reception()
        self.running = True

        # 声明参数
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('init_finish', False)
        self.IMU_FRAME = self.get_parameter('imu_frame').value

        self.imu_pub = self.create_publisher(Imu, '~/imu_raw', 1)
        self.joy_pub = self.create_publisher(Joy, '~/joy', 1)
        self.sbus_pub = self.create_publisher(Sbus, '~/sbus', 1)
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)
        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)
        self.create_subscription(LedState, '~/set_led', self.set_led_state, 5)
        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_state, 5)
        self.create_subscription(OLEDState, '~/set_oled', self.set_oled_state, 5)
        self.create_subscription(MotorsState, '~/set_motor', self.set_motor_state, 10)
        self.create_subscription(RGBsState, '~/set_rgb', self.set_rgb_state, 10)
        self.create_subscription(Bool, '~/enable_reception', self.enable_reception, 1)
        self.create_subscription(SetBusServoState, '~/bus_servo/set_state', self.set_bus_servo_state, 10)
        self.create_subscription(ServosPosition, '~/bus_servo/set_position', self.set_bus_servo_position, 10)
        self.create_subscription(SetPWMServoState, '~/pwm_servo/set_state', self.set_pwm_servo_state, 10)
        self.create_service(GetBusServoState, '~/bus_servo/get_state', self.get_bus_servo_state)
        self.create_service(GetPWMServoState, '~/pwm_servo/get_state', self.get_pwm_servo_state)

        self.board.pwm_servo_set_offset(1, 0)
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        self.clock = self.get_clock()
        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def pub_callback(self):
        while self.running:
            if self.enable_reception:
                self.pub_button_data(self.button_pub)
                self.pub_joy_data(self.joy_pub)
                self.pub_imu_data(self.imu_pub)
                self.pub_sbus_data(self.sbus_pub)
                self.pub_battery_data(self.battery_pub)
                time.sleep(0.02)
            else:
                time.sleep(0.02)
        rclpy.shutdown()

    def enable_reception(self, msg):
        self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_reception ' + str(msg.data)))
        self.enable_reception = msg.data
        self.board.enable_reception(msg.data)

    def set_led_state(self, msg):
        self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)

    def set_buzzer_state(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)

    def set_motor_state(self, msg):
        data = []
        for i in msg.data:
            data.extend([[i.id, i.rps]])
        self.board.set_motor_speed(data)

    def set_oled_state(self, msg):
        self.board.set_oled_text(int(msg.index), msg.text)

    def set_pwm_servo_state(self, msg):
        data = []
        for i in msg.state:
            if i.id and i.position:
                data.extend([[i.id[0], i.position[0]]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])

        if data != []:
            self.board.pwm_servo_set_position(msg.duration, data)
           
    def set_rgb_state(self, msg):
        for i in msg.data:
            self.board.set_rgb([[i.id, i.r, i.g, i.b]])

    def get_pwm_servo_state(self, msg):
        states = []
        for i in msg.cmd:
            data = PWMServoState()
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            states.append(data)
        return [True, states]

    def set_bus_servo_position(self, msg):
        data = []
        for i in msg.position:
            data.extend([[i.id, i.position]])
        if data:
            self.board.bus_servo_set_position(msg.duration, data)

    def set_bus_servo_state(self, msg):
        data = []
        servo_id = []
        for i in msg.state:
            if i.present_id:
                if i.present_id[0]:
                    if i.target_id:
                        if i.target_id[0]:
                            self.board.bus_servo_set_id(i.present_id[1], i.target_id[1])
                    if i.position:
                        if i.position[0]:
                            data.extend([[i.present_id[1], i.position[1]]])
                    if i.offset:
                        if i.offset[0]:
                            self.board.bus_servo_set_offset(i.present_id[1], i.offset[1])
                    if i.position_limit:
                        if i.position_limit[0]:
                            self.board.bus_servo_set_angle_limit(i.present_id[1], i.position_limit[1:])
                    if i.voltage_limit:
                        if i.voltage_limit[0]:
                            self.board.bus_servo_set_vin_limit(i.present_id[1], i.voltage_limit[1:])
                    if i.max_temperature_limit:
                        if i.max_temperature_limit[0]:
                            self.board.bus_servo_set_temp_limit(i.present_id[1], i.max_temperature_limit[1])
                    if i.enable_torque:
                        if i.enable_torque[0]:
                            self.board.bus_servo_enable_torque(i.present_id[1], i.enable_torque[1])
                    if i.save_offset:
                        if i.save_offset[0]:
                            self.board.bus_servo_save_offset(i.present_id[1])
                    if i.stop:
                        if i.stop[0]:
                            servo_id.append(i.present_id[1])
        if data != []:
            self.board.bus_servo_set_position(msg.duration, data)
        if servo_id != []:    
            self.board.bus_servo_stop(servo_id)

    def get_bus_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = BusServoState()
            if i.get_id:
                state = self.board.bus_servo_read_id(i.id)
                if state is not None:
                    i.id = state[0]
                    data.present_id = state
            if i.get_position:
                state = self.board.bus_servo_read_position(i.id)
                if state is not None:
                    data.position = state
            if i.get_offset:
                state = self.board.bus_servo_read_offset(i.id)
                if state is not None:
                    data.offset = state
            if i.get_voltage:
                state = self.board.bus_servo_read_voltage(i.id)
                if state is not None:
                    data.voltage = state
            if i.get_temperature:
                state = self.board.bus_servo_read_temp(i.id)
                if state is not None:
                    data.temperature = state
            if i.get_position_limit:
                state = self.board.bus_servo_read_angle_limit(i.id)
                if state is not None:
                    data.position_limit = state
            if i.get_voltage_limit:
                state = self.board.bus_servo_read_vin_limit(i.id)
                if state is not None:
                    data.voltage_limit = state
            if i.get_max_temperature_limit:
                state = self.board.bus_servo_read_temp_limit(i.id)
                if state is not None:
                    data.max_temperature_limit = state
            if i.get_torque_state:
                state = self.board.bus_servo_read_torque(i.id)
                if state is not None:
                    data.enable_torque = state
            states.append(data)
        response.state = states
        response.success = True
        return response

    def pub_battery_data(self, pub):
        data = self.board.get_battery()
        if data is not None:
            msg = UInt16()
            msg.data = data
            pub.publish(msg)

    def pub_button_data(self, pub):
        data = self.board.get_button()
        if data is not None:
            msg = ButtonState()
            msg.id = data[0]
            msg.state = data[1]
            pub.publish(msg)

    def pub_joy_data(self, pub):
        data = self.board.get_gamepad()
        if data is not None:
            msg = Joy()
            msg.axes = data[0]
            msg.buttons = data[1]
            msg.header.stamp = self.clock.now().to_msg()
            pub.publish(msg)

    def pub_sbus_data(self, pub):
        data = self.board.get_sbus()
        if data is not None:
            msg = Sbus()
            msg.channel = data
            msg.header.stamp = self.clock.now().to_msg()
            pub.publish(msg)

    def pub_imu_data(self, pub):
        data = self.board.get_imu()
        if data is not None:
            ax, ay, az, gx, gy, gz = data
            msg = Imu()
            msg.header.frame_id = self.IMU_FRAME
            msg.header.stamp = self.clock.now().to_msg()

            msg.orientation.w = 0.0
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0

            msg.linear_acceleration.x = ax * self.gravity
            msg.linear_acceleration.y = ay * self.gravity
            msg.linear_acceleration.z = az * self.gravity

            msg.angular_velocity.x = math.radians(gx)
            msg.angular_velocity.y = math.radians(gy)
            msg.angular_velocity.z = math.radians(gz)

            msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            msg.linear_acceleration_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.004]
            pub.publish(msg)

def main():
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == '__main__':
    main()

