import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from rviz_polygon_selection_tool.srv import GetSelection

class Polygon2KeepoutFilter(Node):
    def __init__(self):
        super().__init__('polygon_to_keepout')

        self.no_go_zone_pub = self.create_publisher(MarkerArray, '/no_go_zone', 10)
        self.cli = self.create_client(GetSelection, '/get_selection')

        self.polygon_list = []

        self.timer = self.create_timer(1.0, self._tick)
        self._waiting_logged = False
        self._call_in_flight = False

        self.line_width = 0.05

    def _tick(self):
        if self._call_in_flight:
            return
        
        if not self.cli.service_is_ready():
            if not self._waiting_logged:
                self.get_logger().warn(
                    f'Waiting for service: /get_selection'
                    f'(Is Rviz running with polygon tool loaded?)'
                )
                self._waiting_logged = True
            return
        self._waiting_logged = False
        self._call_in_flight = True

        req = GetSelection.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self._call_in_flight = False
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'GetSelection call failed : {e}')
            return
        
        self.polygon_list = list(resp.selection)
        self.get_logger().info(f'Received {len(self.polygon_list)} polygons from RViz')

        msg = self._build_marker_array(self.polygon_list)
        self.no_go_zone_pub.publish(msg)

    def _build_marker_array(self, polys):
        arr = MarkerArray()

        if len(polys) == 0:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.action = Marker.DELETEALL
            arr.markers.append(m)
            return arr
        
        now = self.get_clock().now().to_msg()

        for i, poly_stamped in enumerate(polys):
            m = Marker()
            m.header = poly_stamped.header
            m.header.stamp = now

            m.ns = 'no_go_zone'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0

            m.scale.x = self.line_width

            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2

            pts32 = poly_stamped.polygon.points
            if len(pts32) < 2:
                continue

            m.points = [Point(x=p.x, y=p.y, z=p.z) for p in pts32]
            if len(m.points) >= 3:
                m.points.append(m.points[0])

            arr.markers.append(m)

        return arr


def main():
    rclpy.init()
    node = Polygon2KeepoutFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()