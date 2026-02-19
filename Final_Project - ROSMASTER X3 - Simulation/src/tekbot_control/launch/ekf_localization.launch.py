from launch import LaunchContext, LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    config_tekbot_ekf = PathJoinSubstitution(
        [FindPackageShare('tekbot_control'),
        'config',
        'localization.yaml'],
    )

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{'use_sim_time': True}, config_tekbot_ekf],
        )
    ld.add_action(node_ekf)

    return ld
