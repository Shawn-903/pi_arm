"""Exclusive CAN owner for bounded motor/joint direction tests. No ros2_control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("websocket_port", default_value="8765"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("pi_arm_bringup"), "launch", "monitor.launch.py",
        ])), launch_arguments={"commissioning": "true"}.items()),
        Node(package="pi_arm_websocket", executable="pi_arm_websocket",
             name="pi_arm_websocket", output="screen",
             parameters=[{"hardware_type": "real", "commissioning": True,
                          "port": LaunchConfiguration("websocket_port")}]),
    ])
