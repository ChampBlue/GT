from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, IncludeLaunchDescription, GroupAction
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='')
    mode_arg = DeclareLaunchArgument('mode', default_value='r')

    ns = LaunchConfiguration('ns')
    params_file = LaunchConfiguration('params_file')

    navigation_pkg = FindPackageShare('xtgt_navigation')

    rotation_params = PathJoinSubstitution([navigation_pkg, 'params', 'rotation_params.yaml'])
    for_back_params = PathJoinSubstitution([navigation_pkg, 'params', 'for_back_params.yaml'])

    set_mode_r = SetLaunchConfiguration('params_file', rotation_params)
    set_mode_f = SetLaunchConfiguration('params_file', for_back_params, condition=LaunchConfigurationEquals('mode', 'f'))

    nav2_group = GroupAction([
        PushRosNamespace(ns),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([navigation_pkg, 'include', 'navigation_launch.py'])
            ),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'true',
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        )
    ])

    return LaunchDescription([
        ns_arg,
        mode_arg,
        set_mode_r,
        set_mode_f,
        nav2_group
    ])