from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    cable_kinematics_file = LaunchConfiguration("cable_kinematics_file")
    calibration_file = LaunchConfiguration("calibration_file")

    description_file = PathJoinSubstitution(
        [FindPackageShare("pi_arm_description"), "urdf", "pi_arm.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_file,
                " backend:=real",
                " can_interface:=",
                can_interface,
                " cable_kinematics_file:=",
                cable_kinematics_file,
                " calibration_file:=",
                calibration_file,
                " allow_real_control:=false",
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("commissioning", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument(
                "cable_kinematics_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("pi_arm_description"),
                        "config",
                        "cable_kinematics.yaml",
                    ]
                ),
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
            ),
            Node(
                package="pi_arm_hardware",
                executable="pi_arm_hardware_monitor_node",
                name="pi_arm_hardware_monitor",
                output="screen",
                parameters=[robot_description, {"commissioning": LaunchConfiguration("commissioning")}],
            ),
        ]
    )
