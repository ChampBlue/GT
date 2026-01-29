from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='')
    rtk_arg = DeclareLaunchArgument('rtk', default_value='d')

    ns = LaunchConfiguration('ns')
    params_file = LaunchConfiguration('params_file')

    params_pkg = FindPackageShare('xtgt_localization')
    dual_param = PathJoinSubstitution([params_pkg, 'params', 'dual_ekf_params.yaml'])
    single_param = PathJoinSubstitution([params_pkg, 'params', 'single_ekf_params.yaml'])
    none_param = PathJoinSubstitution([params_pkg, 'params', 'none_ekf_params.yaml'])

    set_d = SetLaunchConfiguration('params_file', dual_param)
    set_s = SetLaunchConfiguration('params_file', single_param, condition=LaunchConfigurationEquals('rtk', 's'))
    set_n = SetLaunchConfiguration('params_file', none_param, condition=LaunchConfigurationEquals('rtk', 'n'))
    
    navsat_enable = LaunchConfigurationNotEquals('rtk', 'n')

    odom_topic = 'odometry/filtered_map'

    ekf_group = GroupAction([
        PushRosNamespace(ns),

        #ODOM TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            parameters=[params_file,
                        {'odom_frame': [ns, '/odom']},
                        {'base_link_frame': [ns, '/base_footprint']},
                        {'world_frame': [ns, '/odom']}]
        ),
        #MAP TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_map',
            output='screen',
            parameters=[params_file,
                        {'odom_frame': [ns, '/odom']},
                        {'base_link_frame': [ns, '/base_footprint']}
            ],
            remappings=[
                ('odometry/filtered', odom_topic)
            ]
        ),
        #NAVSAT
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('imu', 'gps/imu'),
                ('gps/fix', 'mb/ublox/fix'),
                ('odometry/filtered', odom_topic)
            ],
            condition=navsat_enable,
        )
    ])

    return LaunchDescription([
        ns_arg,
        rtk_arg,
        set_d,
        set_s,
        set_n,
        ekf_group
    ])