from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    tekbot_description_path = PathJoinSubstitution(
        [FindPackageShare("tekbot_description"),
        "launch",
        "description.launch.py"]
    )

    tekbot_description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tekbot_description_path]))

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("tekbot_description"),
        "rviz",
        "tekbot_des.rviz"]
    )

    return LaunchDescription([
        
        tekbot_description_launch,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
