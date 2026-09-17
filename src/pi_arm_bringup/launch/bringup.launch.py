from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    can_interface = LaunchConfiguration("can_interface")
    bitrate = LaunchConfiguration("bitrate")
    cable_kinematics_file = LaunchConfiguration("cable_kinematics_file")
    calibration_file = LaunchConfiguration("calibration_file")
    allow_real_control = LaunchConfiguration("allow_real_control")
    use_rviz = LaunchConfiguration("use_rviz")
    use_manager = LaunchConfiguration("use_manager")
    use_websocket = LaunchConfiguration("use_websocket")
    # controller_manager accepts the first transient-local robot description it
    # receives and ignores later ones.  A stale publisher on /robot_description
    # can therefore select hardware parameters from an older launch.  Keep the
    # control description on a launch-specific configuration path so an
    # explicit backend/interface selection cannot be replaced by that stale
    # global sample.
    controller_robot_description_topic = PathJoinSubstitution(
        ["/pi_arm/controller_robot_description", hardware_type, can_interface]
    )

    description_file = PathJoinSubstitution(
        [FindPackageShare("pi_arm_description"), "urdf", "pi_arm.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_file,
                " backend:=",
                hardware_type,
                " can_interface:=",
                can_interface,
                " bitrate:=",
                bitrate,
                " cable_kinematics_file:=",
                cable_kinematics_file,
                " calibration_file:=",
                calibration_file,
                " allow_real_control:=",
                allow_real_control,
            ]
        )
    }
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("pi_arm_bringup"), "config", "controllers.yaml"]
    )
    moveit_share = Path(get_package_share_directory("pi_arm_moveit_config"))
    robot_description_semantic = {
        "robot_description_semantic": (
            moveit_share / "config" / "pi_arm.srdf"
        ).read_text(encoding="utf-8")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": yaml.safe_load(
            (moveit_share / "config" / "kinematics.yaml").read_text(encoding="utf-8")
        )
    }
    # joint_limits.yaml is the source for joint acceleration overrides (URDF has
    # position/velocity). Pilz TCP caps live in pilz_cartesian_limits.yaml for
    # move_group only; manager MoveL caps are compile-time constants in manager_node.cpp.
    robot_description_planning = {
        "robot_description_planning": yaml.safe_load(
            (moveit_share / "config" / "joint_limits.yaml").read_text(encoding="utf-8")
        )
    }

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pi_arm_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments={
            "hardware_type": hardware_type,
            "can_interface": can_interface,
            "bitrate": bitrate,
            "cable_kinematics_file": cable_kinematics_file,
            "calibration_file": calibration_file,
            "allow_real_control": allow_real_control,
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pi_arm_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        ),
        launch_arguments={
            "hardware_type": hardware_type,
            "can_interface": can_interface,
            "bitrate": bitrate,
            "cable_kinematics_file": cable_kinematics_file,
            "calibration_file": calibration_file,
            "allow_real_control": allow_real_control,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_type",
                default_value="mock",
                choices=["mock", "real"],
                description="Select deterministic mock or SocketCAN real hardware",
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("bitrate", default_value="1000000"),
            DeclareLaunchArgument(
                "cable_kinematics_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("pi_arm_description"),
                        "config",
                        "cable_kinematics.yaml",
                    ]
                ),
                description="Installed six-axis cable kinematics YAML",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("pi_arm_hardware"),
                        "config",
                        "pack_reference_calibration.template.yaml",
                    ]
                ),
                description="Pack-reference actuator calibration YAML",
            ),
            DeclareLaunchArgument(
                "allow_real_control",
                default_value="false",
                choices=["true", "false"],
                description="Explicitly allow real CAN position commands after calibration gates pass",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_manager", default_value="true"),
            DeclareLaunchArgument("use_websocket", default_value="true"),
            DeclareLaunchArgument("websocket_port", default_value="8765"),
            DeclareLaunchArgument("points_file", default_value=""),
            DeclareLaunchArgument("manager_config", default_value=PathJoinSubstitution(
                [FindPackageShare("pi_arm_bringup"), "config", "manager.yaml"])),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_link",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "world", "--child-frame-id", "base_link",
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
                remappings=[
                    ("robot_description", controller_robot_description_topic)
                ],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[controllers_file],
                remappings=[
                    ("robot_description", controller_robot_description_topic)
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            ),
            move_group,
            Node(
                package="pi_arm_manager",
                executable="pi_arm_manager_node",
                # Do not set name= here. MoveGroupInterface creates a second
                # rclcpp::Node inside this process; launch's name remapping would
                # rename both to pi_arm_manager and produce a duplicate graph entry.
                # The C++ node is already constructed as Node("pi_arm_manager").
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    {
                        "still_velocity_rad_s": 0.00017453292519943296,
                        "follow_joint_trajectory_action":
                            "/joint_trajectory_controller/follow_joint_trajectory",
                    },
                    LaunchConfiguration("manager_config"),
                ],
                condition=IfCondition(use_manager),
            ),
            Node(
                package="pi_arm_websocket",
                executable="pi_arm_websocket",
                name="pi_arm_websocket",
                output="screen",
                parameters=[{"hardware_type": hardware_type, "calibration_file": calibration_file,
                             "points_file": ParameterValue(LaunchConfiguration("points_file"), value_type=str), "port": LaunchConfiguration("websocket_port")}],
                condition=IfCondition(use_websocket),
            ),
            rviz,
        ]
    )
