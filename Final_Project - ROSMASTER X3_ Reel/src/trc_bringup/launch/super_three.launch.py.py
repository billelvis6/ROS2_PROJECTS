import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_trc = FindPackageShare('trc_bringup')
    pkg_nav = FindPackageShare('my_nav_pkg')

    robot_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_trc.find('trc_bringup'), 'launch', 'robot_real_launch.py')
        ])
    )

    nav_and_algo = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_nav.find('my_nav_pkg'), 'launch', 'x3_nav.launch.py')
                ])
            ),
            Node(
                package='my_nav_pkg',
                executable='algo_win_test.py',
                output='screen'
            )
        ]
    )

    return LaunchDescription([robot_real, nav_and_algo])
