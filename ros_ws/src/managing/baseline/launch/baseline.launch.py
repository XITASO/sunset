from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='baseline',
                executable='baseline_node',
                parameters=[{
                    "use_sim_time": True,
                }]
            )
        ]
    )
