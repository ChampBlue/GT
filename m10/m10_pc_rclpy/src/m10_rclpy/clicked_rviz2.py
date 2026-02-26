import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class ClickedRvix2(Node):
    def __init__(self):
        super().__init__('clicked_rviz2_node')

        self.declare_parameter('ns', '')
        self.ns = self.get_parameter('ns').get_parameter_value().string_value

        self.block_poses = []

        self.mark_pub = self.create_publisher(MarkerArray, f'{self.ns}/blocks_marker', 10)

        self.create_subscription(PoseStamped, f"{self.ns}/map_blocks", self._on_add, 10)

    def _on_add(self, msg: PoseStamped):
        self.block_poses.append(msg)
        self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for i, p in enumerate(self.block_poses):
            m = Marker()
            m.header.frame_id = p.header.frame_id or 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f"{self.ns}/blocks_point"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = p.pose
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.a = 1.0; m.color.r = 0.2; m.color.g = 0.9; m.color.b = 0.2
            arr.markers.append(m)
        if self.block_poses:
            line = Marker()
            line.header.frame_id = self.block_poses[0].header.frame_id or 'map'
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = f"{self.ns}/blocks_path"
            line.id = 99999
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color.a = 1.0; line.color.b = 1.0
            line.points = [p.pose.position for p in self.block_poses]
            arr.markers.append(line)
        self.mark_pub.publish(arr)


def main():
    rclpy.init()
    node = ClickedRvix2()
    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()        