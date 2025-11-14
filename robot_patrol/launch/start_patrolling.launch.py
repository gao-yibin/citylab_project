import launch
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Nodes
    patrol_node = Node(
        package='robot_patrol',
        executable='robot_patrol_node',
        arguments=[],
        output='screen',
    )

    RViz = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz', 
        output='screen', 
        arguments=['-d', '/home/user/ros2_ws/src/citylab_project/robot_patrol/rviz2/robot_patrol.rviz'],
    )

    return launch.LaunchDescription([
        patrol_node, RViz
    ])

    TimerAction(
            period=5.0,  # delay in seconds
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2'
                ),
            ]
        )