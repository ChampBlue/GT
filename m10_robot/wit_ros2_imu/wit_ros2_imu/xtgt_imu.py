#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

key = 0
flag = 0
buff = {}
angularVelocity = [0.0, 0.0, 0.0]  
acceleration   = [0.0, 0.0, 0.0]  
magnetometer   = [0.0, 0.0, 0.0]
angle_degree   = [0.0, 0.0, 0.0] 

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def hex_to_short(raw_data):
    return list(struct.unpack("<hhhh", bytearray(raw_data)))

def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity
    angle_flag = False
    buff[key] = raw_data

    key += 1
    if buff.get(0) != 0x55:
        key = 0
        return
    if key < 11:
        return
    else:
        data_buff = list(buff.values())
        if buff[1] == 0x51:
            if check_sum(data_buff[0:10], data_buff[10]):
                acc_raw = hex_to_short(data_buff[2:10])
                for i in range(3):
                    acceleration[i] = acc_raw[i] / 32768.0 * 16.0 * 9.8
            else:
                print('0x51 Check failure')

        elif buff[1] == 0x52:
            if check_sum(data_buff[0:10], data_buff[10]):
                gyro_raw = hex_to_short(data_buff[2:10])
                for i in range(3):
                    angularVelocity[i] = gyro_raw[i] / 32768.0 * 2000.0 * math.pi / 180.0
            else:
                print('0x52 Check failure')

        elif buff[1] == 0x53:
            if check_sum(data_buff[0:10], data_buff[10]):
                # roll, pitch, yawNorth (deg)
                ang_raw = hex_to_short(data_buff[2:10])
                for i in range(3):
                    angle_degree[i] = ang_raw[i] / 32768.0 * 180.0
                angle_flag = True  
            else:
                print('0x53 Check failure')

        elif buff[1] == 0x54:
            if check_sum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
            else:
                print('0x54 Check failure')
        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
        return angle_flag

# ── 쿼터니언 변환 (원 코드 유지) ──────────────────────────────────────────
def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

class IMUDriverNode(Node):
    def __init__(self, port_name):
        super().__init__('imu_driver_node')

        self.declare_parameter('frame_id', 'imu_data')
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id 

        self.declare_parameter('yaw_offset_rad', 0.0)
        self.yaw_offset_rad = float(self.get_parameter('yaw_offset_rad').value or 0.0)

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        self.driver_thread = threading.Thread(target=self.driver_loop, args=(port_name,))
        self.driver_thread.start()

        self._print_count = 0

    def driver_loop(self, port_name):
        try:
            wt_imu = serial.Serial(port=port_name, baudrate=9600, timeout=0.5)
            if wt_imu.isOpen():
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
            else:
                wt_imu.open()
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
        except Exception as e:
            print(e)
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            return 

        while rclpy.ok():
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
                return 
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        tag = handle_serial_data(buff_data[i])
                        if tag:
                            self.imu_data()

    def imu_data(self):
        accel_x, accel_y, accel_z = acceleration[0], acceleration[1], acceleration[2]

        gyro_x, gyro_y, gyro_z = angularVelocity[0], angularVelocity[1], angularVelocity[2]

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z
        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = gyro_z

        roll_rad  = math.radians(angle_degree[0])
        pitch_rad = math.radians(angle_degree[1])
        yaw_north_rad = math.radians(angle_degree[2])

        yaw_enu = wrap_pi(math.pi/2.0 - yaw_north_rad)
        if abs(self.yaw_offset_rad) > 1e-12:
            yaw_enu = wrap_pi(yaw_enu + self.yaw_offset_rad) 

        qx, qy, qz, qw = get_quaternion_from_euler(roll_rad, pitch_rad, yaw_enu)
        self.imu_msg.orientation.x = qx
        self.imu_msg.orientation.y = qy
        self.imu_msg.orientation.z = qz
        self.imu_msg.orientation.w = qw

        self.imu_msg.orientation_covariance = [
            float((math.radians(5.0))**2), 0.0, 0.0,
            0.0, float((math.radians(5.0))**2), 0.0,
            0.0, 0.0, float((math.radians(25.0))**2)
        ]
        self.imu_msg.angular_velocity_covariance = [
            1e-3, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3
        ]
        self.imu_msg.linear_acceleration_covariance = [
            5e-2, 0.0, 0.0,
            0.0, 5e-2, 0.0,
            0.0, 0.0, 5e-2
        ]

        self.imu_pub.publish(self.imu_msg)

        # if self._print_count < 10:
        #     self._print_count += 1
        #     self.get_logger().info(
        #         f"roll={angle_degree[0]:6.2f}° pitch={angle_degree[1]:6.2f}° "
        #         f"yawNorth={angle_degree[2]:7.2f}° → yawENU={math.degrees(yaw_enu):7.2f}° "
        #         f"(offset={math.degrees(self.yaw_offset_rad):.2f}°)"
        #     )

def main():
    rclpy.init()
    node = IMUDriverNode('/dev/ttyIMU')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
