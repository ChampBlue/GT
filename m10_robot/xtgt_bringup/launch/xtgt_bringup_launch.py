from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription

def generate_launch_description():
    robot_pkg = FindPackageShare('xtgt_motor')
    lidar_pkg = FindPackageShare('sllidar_ros2')
    imu_pkg = FindPackageShare('wit_ros2_imu')
    gps_pkg = FindPackageShare('ublox_gps')
    xacro_file = PathJoinSubstitution([FindPackageShare('xtgt_bringup'), 'urdf', 'p05.urdf.xacro'])

    ns_arg = DeclareLaunchArgument('ns', default_value='')
    rtk_arg = DeclareLaunchArgument('rtk', default_value='d')
    ns = LaunchConfiguration('ns')
    rtk = LaunchConfiguration('rtk')

    xtgt_group = GroupAction([
        PushRosNamespace(ns),

        #URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    xacro_file,
                    ' ',
                    'prefix:=',
                    ns,
                    '/'
                ])
            }],
        ),

        #LIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([lidar_pkg, 'launch', 'xtgt_lidar_launch.py'])
            ),
            launch_arguments={'frame_id': [ns, '/laser']}.items()
        ),

        #IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([imu_pkg, 'launch', 'xtgt_imu_launch.py'])
            )
        ),

        #GPS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gps_pkg, 'launch', 'xtgt_gps_launch.py'])
            ),
            launch_arguments={'rtk': rtk, 'ns': ns}.items()
        ),

        # #MOTOR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([robot_pkg, 'launch', 'xtgt_motor_launch.py'])
            )
        ),

        #LIDAR FILTER
        Node(
            package='xtgt_bringup',
            executable='lidar_filter',
            name='lidar_filter',
            output='screen',
            parameters=[{
                'frame_id': [ns, '/laser'],
                'min_distance_threshold': 3.0
            }]
        ),
    ])

    return LaunchDescription([
        ns_arg,
        rtk_arg,
        xtgt_group
    ])