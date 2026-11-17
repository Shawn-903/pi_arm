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
    # controller_manager 使用最先收到的持久化模型消息，后到的描述不会替换它。
    # 因此按后端类型和 CAN 接口隔离描述话题，避免 /robot_description 上残留的
    # 旧发布者覆盖本次启动选定的硬件参数。
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
                " software_zero_file:='",
                LaunchConfiguration("software_zero_file"),
                "'",
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
    # URDF 提供关节位置/速度限位，joint_limits.yaml 补充加速度。
    # Pilz 末端限速由 move_group 的 pilz_cartesian_limits.yaml 配置；
    # manager 的 MoveL 请求上限另在 manager_node.cpp 中定义，修改时需核对两处。
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
            DeclareLaunchArgument("software_zero_file", default_value=""),
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
                # 此处不设置 name：MoveGroupInterface 会在进程内创建另一个节点，
                # launch 名称重映射会把两者改成同名。C++ 主节点已有 pi_arm_manager 名称。
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
