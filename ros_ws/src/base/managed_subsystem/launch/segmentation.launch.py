from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="managed_subsystem",
                namespace="managed_subsystem",
                executable="segmentation",
                name="segmentation",
                parameters=[{
                    "use_sim_time": True,
                }]
            ),
        ]
    )
