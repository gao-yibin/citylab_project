import launch
from launch_ros.actions import Node


def generate_launch_description():
    # Nodes
    test_service_node = Node(
        package='robot_patrol',
        executable='test_service_executable',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        test_service_node
    ])