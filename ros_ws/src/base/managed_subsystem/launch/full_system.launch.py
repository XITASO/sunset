from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("managed_subsystem"),
                                "launch",
                                "camera.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("managed_subsystem"),
                                "launch",
                                "depth.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("managed_subsystem"),
                                "launch",
                                "image_enhancement.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("managed_subsystem"),
                                "launch",
                                "sensor_fusion.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("managed_subsystem"),
                                "launch",
                                "segmentation.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                       PathJoinSubstitution(
                            [
                                FindPackageShare("managed_subsystem"),
                                "launch",
                                "abstract_blackboard_setter.launch.py",
                            ]
                        )
                    ]
                ),
            ),
        ]
    )
