from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = FindPackageShare('bt_mape_k')
    bts_path = PathJoinSubstitution([
        pkg_share,
        'bts'
        ])

    folder_arg = DeclareLaunchArgument(
        'file_folder',
        default_value=bts_path,
        description='The folder path to be passed to the node'
    )
    return LaunchDescription(
        [
            Node(
                package="bt_mape_k",
                namespace="test_ns",
                executable="bt_executor",
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
                kwargs=[bts_path]

            ),
            folder_arg
        ]
    )
