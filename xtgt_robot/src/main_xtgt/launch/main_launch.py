from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_pkg = FindPackageShare('xtgt_bringup')
    RL_pkg = FindPackageShare('xtgt_localization')
    nav_pkg = FindPackageShare('xtgt_navigation')

    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='xtgt2',
        description='Robot namespace'
    )
    rtk_arg = DeclareLaunchArgument(
        'rtk',
        default_value='d',
        description='rtk type'
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='r',
        description='controller type'
    )
    
    ns = LaunchConfiguration('ns')
    rtk = LaunchConfiguration('rtk')
    mode = LaunchConfiguration('mode')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_pkg, 'launch', 'xtgt_bringup_launch.py'])
        ),
        launch_arguments={'ns': ns, 'rtk': rtk}.items()
    )
    RL_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([RL_pkg, 'launch', 'xtgt_localization_launch.py'])
        ),
        launch_arguments={'ns': ns, 'rtk': rtk}.items()
    )
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_pkg, 'launch', 'xtgt_navigation_launch.py'])
        ),
        launch_arguments={'ns': ns, 'mode': mode}.items()
    ) 

    return LaunchDescription([
        ns_arg,
        mode_arg,
        rtk_arg,
        LogInfo(msg=[TextSubstitution(text='[main] ns='), ns]),
        bringup_launch,
        TimerAction(period=2.0, actions=[RL_launch]),
        TimerAction(period=8.0, actions=[nav_launch])
    ])