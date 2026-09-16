"""Real feedback to the web UI, without a motion/control chain."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("pi_arm_bringup"), "launch", "monitor.launch.py",
            ])
        )),
        Node(
            package="pi_arm_websocket", executable="pi_arm_websocket",
            name="pi_arm_websocket", output="screen",
            parameters=[{"hardware_type": "real", "read_only": True}],
        ),
    ])
