import launch
from launch_ros.actions import Node


def generate_launch_description():
    # Nodes
    direction_service_node = Node(
        package='robot_patrol',
        executable='direction_service_node',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        direction_service_node
    ])