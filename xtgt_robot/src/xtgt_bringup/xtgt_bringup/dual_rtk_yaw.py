#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from ublox_msgs.msg import NavRELPOSNED9


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class GpsHeading2Imu(Node):
    def __init__(self):
        super().__init__('dual_rtk_yaw')

        # ---- params ----
        self.declare_parameter('relpos_topic', 'rv/navrelposned')
        self.declare_parameter('imu_topic', 'gps/imu')
        self.declare_parameter('yaw_topic', 'gps/yaw')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('mount_yaw_offset_rad', 0.0)

        # 0: no check, 1: FLOAT 이상, 2: FIXED만(=정수)
        self.declare_parameter('min_carr_soln', 2)
        # 보통 true 유지 추천
        self.declare_parameter('require_gnss_fix_ok', True)

        self.relpos_topic = self.get_parameter('relpos_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.yaw_topic = self.get_parameter('yaw_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mount_offset = float(self.get_parameter('mount_yaw_offset_rad').value)
        self.min_carr_soln = int(self.get_parameter('min_carr_soln').value)
        self.require_gnss_fix_ok = bool(self.get_parameter('require_gnss_fix_ok').value)

        self.pub_imu = self.create_publisher(Imu, self.imu_topic, qos_profile_sensor_data)
        self.pub_yaw = self.create_publisher(Float32, self.yaw_topic, 10)

        self.create_subscription(
            NavRELPOSNED9,
            self.relpos_topic,
            self.cb_relposned,
            qos_profile_sensor_data
        )

    def cb_relposned(self, msg: NavRELPOSNED9):
        flags = int(msg.flags)

        # u-blox UBX-NAV-RELPOSNED flags (bit positions)
        # bit0: gnssFixOK, bit2: relPosValid, bits4..3: carrSoln(0/1/2), bit8: relPosHeadingValid
        gnss_fix_ok = bool(flags & 0x01)
        relpos_valid = bool(flags & 0x04)
        head_valid = bool(flags & 0x100)
        carr_soln = (flags & 0x18) >> 3  # 0/1/2
        #carr_soln = (flags >> 3) & 0x03

        ok = relpos_valid and head_valid
        if self.require_gnss_fix_ok:
            ok = ok and gnss_fix_ok
        if self.min_carr_soln >= 0:
            ok = ok and (carr_soln >= self.min_carr_soln)

        if not ok:
            return

        # heading: 1e-5 deg, north=0°, clockwise positive (east=90°)
        heading_deg = float(msg.rel_pos_heading) * 1e-5
        heading_rad = math.radians(heading_deg)

        # ENU yaw: east=0, CCW positive => yaw = pi/2 - heading
        yaw_enu = wrap_pi((math.pi * 0.5) - heading_rad + self.mount_offset)

        # accuracy: 1e-5 deg -> rad -> variance
        acc_deg = float(msg.acc_heading) * 1e-5
        acc_rad = math.radians(acc_deg)
        var_yaw = max(acc_rad * acc_rad, 1e-8)

        now = self.get_clock().now()

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.frame_id

        qx, qy, qz, qw = quat_from_yaw(yaw_enu)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, var_yaw
        ]
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(imu)
        self.pub_yaw.publish(Float32(data=yaw_enu))


def main():
    rclpy.init()
    node = GpsHeading2Imu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
