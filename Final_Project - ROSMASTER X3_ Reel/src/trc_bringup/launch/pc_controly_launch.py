import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 1. Chemin vers ton fichier RViz
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('my_nav_pkg'),
        'rviz',
        'trc.rviz'
    ])

    # 2. Chemin CORRIGÉ vers tes paramètres SLAM dans my_nav_pkg
    # Assure-toi que le fichier s'appelle bien mapper_params_online_async.yaml dans ton pkg
    slam_params = PathJoinSubstitution([
        FindPackageShare('my_nav_pkg'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    # Nœud Joy (Manette)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'device_id': 0
        }],
        output='screen'
    )

    # Nœud Teleop (Contrôle X3)
    teleop_node = Node(
        package='x3_control',
        executable='yahboom_joy_X3.py',
        name='joy_X3',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

       # Nœud RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        joy_node,
        teleop_node,
        rviz_node
    ])
