from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    
    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare('tekbot_control'), 'config', ('teleop_mactech.yaml')]
    )
    
    
    node_joy = Node(
        namespace='joy_teleop',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy]
    )

    node_teleop_twist_joy = Node(
        namespace='joy_teleop',
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        remappings={('/joy_teleop/cmd_vel', '/cmd_vel')},
        parameters=[filepath_config_joy]
    )


    ld = LaunchDescription()
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    return ld
