from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    cable_kinematics_file = LaunchConfiguration("cable_kinematics_file")
    model = PathJoinSubstitution(
        [FindPackageShare("pi_arm_description"), "urdf", "pi_arm.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                model,
                " backend:=",
                backend,
                " cable_kinematics_file:=",
                cable_kinematics_file,
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="mock",
                choices=["mock", "real"],
                description="ros2_control hardware backend",
            ),
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
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
            ),
        ]
    )
