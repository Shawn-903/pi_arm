"""装配维护监测节点、方向测试、软件标零任务和网页网关，由监测节点独占 CAN。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("websocket_port", default_value="8765"),
        DeclareLaunchArgument("points_file", default_value=""),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("pi_arm_bringup"), "launch", "monitor.launch.py",
        ])), launch_arguments={"commissioning": "true"}.items()),
        Node(package="pi_arm_tasks", executable="pi_arm_tasks", output="screen"),
        Node(package="pi_arm_websocket", executable="pi_arm_websocket",
             name="pi_arm_websocket", output="screen",
             parameters=[{"hardware_type": "real", "commissioning": True,
                          "calibration_file": LaunchConfiguration("calibration_file"),
                          "points_file": ParameterValue(LaunchConfiguration("points_file"), value_type=str), "port": LaunchConfiguration("websocket_port")}]),
    ])
