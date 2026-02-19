import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    # 1. Configuration des variables et chemins
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav_pkg_share = FindPackageShare('my_nav_pkg')
    
    params_file = PathJoinSubstitution([nav_pkg_share, 'config', 'nav2_params.yaml'])
    rviz_config_file = PathJoinSubstitution([nav_pkg_share, 'rviz', 'trc_nav.rviz'])
    map_file = PathJoinSubstitution([nav_pkg_share, 'maps', 'map_win.yaml'])

    # 2. DÉCLARATION DES NŒUDS
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_file}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'robot_model_type': "nav2_amcl::OmniMotionModel",
            ############# SOLUTION 2 : On définit la pose initiale ici
           # 'set_initial_pose': True,
           # 'initial_pose': {
           #     'x': 0.0, 
           #     'y': 0.0, 
           #     'z': 0.0, 
           #     'yaw': 0.0
           # }
        }]
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 3. ACTIONS D'ACTIVATION (Lifecycle)
    # Note: On garde les délais pour laisser le temps à la Pi 4 de respirer
    activate_map = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
        output='screen'
    )

    activate_amcl = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
        output='screen'
    )

    activate_planner = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'planner_server'],
        output='screen'
    )

    activate_controller = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'controller_server'],
        output='screen'
    )

    # 4. DESCRIPTION DU LANCEMENT
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        rviz_node,

        TimerAction(period=2.0, actions=[activate_map]),
        TimerAction(period=4.0, actions=[activate_amcl]),
        TimerAction(period=6.0, actions=[activate_planner]),
        TimerAction(period=8.0, actions=[activate_controller]),
    ])
