from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pi_arm_tasks.zero_reference.files import configure_launch_references


def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")
    cable_kinematics_file = LaunchConfiguration("cable_kinematics_file")

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
                " ",
                LaunchConfiguration("reference_xacro_args"),
                " allow_real_control:=false",
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("software_zero_file", default_value=""),
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
            DeclareLaunchArgument("calibration_file", default_value=""),
            OpaqueFunction(function=configure_launch_references, kwargs={"monitor": True}),
            Node(
                package="pi_arm_hardware",
                executable="pi_arm_hardware_monitor_node",
                name="pi_arm_hardware_monitor",
                output="screen",
                parameters=[robot_description, {"commissioning": LaunchConfiguration("commissioning")}],
            ),
        ]
    )
