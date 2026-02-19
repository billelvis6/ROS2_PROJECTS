import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_trc = FindPackageShare('trc_bringup')

    robot_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_trc.find('trc_bringup'), 'launch', 'robot_real_launch.py')
        ])
    )

    pc_control = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_trc.find('trc_bringup'), 'launch', 'pc_control_launch.py')
                ])
            )
        ]
    )

    return LaunchDescription([robot_real, pc_control])
