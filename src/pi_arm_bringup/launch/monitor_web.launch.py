"""Real feedback to the web UI, without a motion/control chain."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("points_file", default_value=""),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("pi_arm_bringup"), "launch", "monitor.launch.py",
            ])
        )),
        Node(
            package="pi_arm_websocket", executable="pi_arm_websocket",
            name="pi_arm_websocket", output="screen",
            parameters=[{"hardware_type": "real", "read_only": True, "points_file": ParameterValue(LaunchConfiguration("points_file"), value_type=str),
                         "calibration_file": LaunchConfiguration("calibration_file")}],
        ),
    ])
