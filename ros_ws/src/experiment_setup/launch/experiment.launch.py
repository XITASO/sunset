from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    PathJoinSubstitution,
)


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
                                "full_system.launch.py",
                            ]
                        )
                    ]
                ),
            ),
            Node(
                package="experiment_setup",
                namespace="experiment_setup",
                executable="evaluator",
                name="evaluator",
                remappings=[
                    (
                        "/experiment_setup/segmentation",
                        "/managed_subsystem/segmentation",
                    ),
                    ("/experiment_setup/gt_segmentation", "/gt_segmentation"),
                ],
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="experiment_setup",
                namespace="experiment_setup",
                executable="experiment_logger",
                name="experiment_logger",
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
                remappings=[
                    (
                        "/experiment_setup/camera/experiment_log",
                        "/managed_subsystem/camera/experiment_log",
                    ),
                    (
                        "/experiment_setup/depth/experiment_log",
                        "/managed_subsystem/depth/experiment_log",
                    ),
                ],
            )
        ]
    )
