import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    TRC_WS = os.environ.get('TRC_WS', '~/trc_ws/src')

    # LiDAR A1
    rplidar_launch = os.path.join(
        TRC_WS,
        '2025-ROS-Ressources',
        'drivers',
        'rplidar_ros',
        'launch',
        'rplidar_a1_launch.py'
    )

    # Caméra Astra
    astra_launch = os.path.join(
        TRC_WS,
        '2025-ROS-Ressources',
        'drivers',
        'ros2_astra_camera',
        'astra_camera',
        'launch',
        'astra.launch.xml'
    )

    # Description robot
    x3_desc_launch = os.path.join(
        TRC_WS,
        '2025-ROS-Ressources',
        'rosmaster',
        'x3_description',
        'launch',
        'x3_description.launch.py'
    )

    return LaunchDescription([

        # LiDAR A1
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rplidar_launch)
        ),

        # Caméra Astra
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch)
        ),

        # Description robot
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(x3_desc_launch)
        ),

        # Téléop manette
        Node(
            package='x3_control',
            executable='yahboom_joy_X3.py',
            name='joy_node',
            output='screen'
        ),
    ])

