from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    tekbot_urdf = PathJoinSubstitution(
        [FindPackageShare("tekbot_description"),
        "urdf",
        "tekbot.urdf"]
    )

    robot_description = ParameterValue(Command(['xacro ', tekbot_urdf]),
                                       value_type=str)

    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}])
    ])