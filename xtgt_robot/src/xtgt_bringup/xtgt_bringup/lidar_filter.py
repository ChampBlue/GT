#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')

        self.declare_parameter('min_distance_threshold', 0.5)
        self.declare_parameter('frame_id', 'laser')

        self.min_distance_threshold = float(self.get_parameter('min_distance_threshold').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.add_on_set_parameters_callback(self._on_param)

        self.scan_pub = self.create_publisher(LaserScan, 'filtered_scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)

    def _on_param(self, params):
        for p in params:
            if p.name == 'min_distance_threshold':
                self.min_distance_threshold = float(p.value)
                self.get_logger().info(f"lidar min distance -> {self.min_distance_threshold}")
        
        return SetParametersResult(successful=True)

    def scan_cb(self, msg):
        filtered_ranges = []

        for r in msg.ranges:
            if r < self.min_distance_threshold:
                filtered_ranges.append(0.0)
            else:
                filtered_ranges.append(r)

        msg.ranges = filtered_ranges
        msg.header.frame_id = self.frame_id
        self.scan_pub.publish(msg)

def main():
    rclpy.init()
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
       node.get_logger().info("Keyboard Interuup (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
