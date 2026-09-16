from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real", choices=["real", "mock"]),
            Node(
                package="pi_arm_websocket",
                executable="pi_arm_websocket",
                name="pi_arm_websocket",
                output="screen",
                parameters=[
                    {
                        "port": 8765,
                        "hardware_type": LaunchConfiguration("hardware_type"),
                        "state_push_interval_ms": 100,
                        "max_size_bytes": 32 * 1024 * 1024,
                    }
                ],
            )
        ]
    )
