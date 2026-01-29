from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='')
    ns = LaunchConfiguration('ns')

    ip_arg = DeclareLaunchArgument('ip', default_value='192.168.0.20')
    port_arg = DeclareLaunchArgument('port', default_value='9902')
    ip = LaunchConfiguration('ip')
    port = LaunchConfiguration('port')

    motor_node = Node(
        package='xtgt_motor',
        executable='p05_motor',
        name='motor',
        output='screen',
        parameters=[{
            'odom_frame_id': [ns, '/odom'],
            'base_footprint_frame_id': [ns, '/base_footprint'],
            'ip': ip,
            'port': port
        }]
    )

    return LaunchDescription([
        ip_arg,
        port_arg,
        ns_arg,
        motor_node
    ])