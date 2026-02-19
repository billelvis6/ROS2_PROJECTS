import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # Chemin de la configuration de nav2
    params_file = "/home/billy/trc_ws/src/my_nav_pkg/config/nav2_params.yaml"

    # Chemin du fichier RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("my_nav_pkg"),
        "rviz",
        "robot_view_yes.rviz"
    ])

    # Chemin de la carte YAML
    my_map = "/home/billy/trc_ws/src/maps/trc_arena.yaml"

    # -------------------------------
    # Lancement AMCL après 5s
    # -------------------------------
    nav2_launch_localization = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("nav2_bringup"),
                        "launch",
                        "localization_launch.py"
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': my_map,
                }.items()
            )
        ]
    )

    # -------------------------------
    # Lancement navigation après 10s
    # -------------------------------
    nav2_launch_navigation = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("nav2_bringup"),
                        "launch",
                        "navigation_launch.py"
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': params_file,
                    'map_subscriber_transient_local': 'true'
                }.items()
            )
        ]
    )

    # -------------------------------
    # Lancement RViz après 12s
    # -------------------------------
    rviz_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=['-d', rviz_config_file]
            )
        ]
    )

    # -------------------------------
    # Création de la Launch Description
    # -------------------------------
    ld = LaunchDescription([
        nav2_launch_localization,
        nav2_launch_navigation,
        rviz_node,
    ])

    return ld

