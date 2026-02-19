import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('x_init',
            default_value='0.0',
            description='robot initial x'),
    DeclareLaunchArgument('y_init',
            default_value='0.0',
            description='robot initial y'),
    DeclareLaunchArgument('z_init',
            default_value='0.0',
            description='robot initial z'),
    DeclareLaunchArgument('yaw_init',
            default_value='0.0',
            description='robot initial yaw')
]

def generate_launch_description():

  tekbot_description_path = PathJoinSubstitution(
        [FindPackageShare("tekbot_description"),
        "launch",
        "description.launch.py"]
  )

  tekbot_description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tekbot_description_path]))

  # Pose we want to spawn the robot
  spawn_x_val = LaunchConfiguration('x_init')
  spawn_y_val = LaunchConfiguration('y_init')
  spawn_z_val = LaunchConfiguration('z_init')
  spawn_yaw_val = LaunchConfiguration('yaw_init')

  gazebo = ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so',],
            output='screen')

  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', "tekbot", 
                '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen',
    parameters=[{'use_sim_time': True}])
  
  # Create the launch description and populate
  ld = LaunchDescription(ARGUMENTS)

  # Add any actions
  ld.add_action(spawn_entity_cmd)
  ld.add_action(gazebo)
  ld.add_action(tekbot_description_launch)

  return ld
