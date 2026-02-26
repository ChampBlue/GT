from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='')
    ns = LaunchConfiguration('ns')

    imu_node = Node(
        package='wit_ros2_imu',
        executable='xtgt_imu',
        name='imu',
        output='screen',
        parameters=[{
            'frame_id': [ns, '/imu_link']
        }]
    )

    return LaunchDescription([
        ns_arg,
        imu_node
    ])