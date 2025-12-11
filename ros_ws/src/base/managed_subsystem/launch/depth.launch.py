from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="managed_subsystem",
                namespace="managed_subsystem",
                executable="depth",
                name="depth",
                remappings=[
                    ("/managed_subsystem/depth_raw", "/depth_raw"),
                ],
                parameters=[{
                    "use_sim_time": True,
                }]
            ),
        ]
    )
