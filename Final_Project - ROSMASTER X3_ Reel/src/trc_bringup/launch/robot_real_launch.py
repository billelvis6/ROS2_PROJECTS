#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Configuration du temps de simulation (utile si on passe sur Gazebo plus tard)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Récupération du dossier du package Yahboom
    pkg_yahboom_bringup = get_package_share_directory('yahboomcar_bringup')

    # 1. Bringup Yahboom (Drivers moteurs, IMU, Robot State Publisher)
    # On désactive la TF d'origine ('pub_odom_tf': 'false') car robot_localization s'en occupe
    launch_yahboom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yahboom_bringup, 'launch', 'yahboomcar_bringup_X3_launch.py')
        ),
        launch_arguments={
            'pub_odom_tf': 'false',
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. Node RPLidar
    # Le frame_id est 'laser_link' conformément à ton fichier URDF/Xacro
    node_lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200, 
            'frame_id': 'laser_sensor_frame',     
            'angle_compensate': True,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 3. Joy Node (Lecture de la manette)
    joy_node = Node(
        package='joy', 
        executable='joy_node', 
        name='joy_node',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'device_id': 0
        }]
    )

    # 4. Teleop Node (Conversion des boutons en commandes de mouvement)
    teleop_node = Node(
        package='x3_control', 
        executable='yahboom_joy_X3.py', 
        name='joy_X3',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    node_fix_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fix_lidar',
        # On force base_link -> laser_link avec 180°
        arguments = ['0', '0', '0', '3.14159', '0', '0', 'base_link', 'laser_sensor_frame']
    )

    return LaunchDescription([
        # Déclaration des arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Lancement des composants
        launch_yahboom,
        joy_node,
        teleop_node,
        node_fix_lidar,
      
        # On attend 4 secondes avant de lancer le Lidar pour stabiliser le bus USB
        TimerAction(period=4.0, actions=[node_lidar]),
    ])
