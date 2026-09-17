from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cable_kinematics_file = LaunchConfiguration("cable_kinematics_file")
    output_shaft_topic = LaunchConfiguration("output_shaft_topic")
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    arm_model = PathJoinSubstitution(
        [FindPackageShare("pi_arm_description"), "urdf", "pi_arm.urdf.xacro"]
    )
    gui_generator = PathJoinSubstitution(
        [
            FindPackageShare("pi_arm_output_visualizer"),
            "scripts",
            "generate_output_shaft_urdf.py",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("pi_arm_output_visualizer"),
            "config",
            "output_shaft_visualizer.rviz",
        ]
    )

    arm_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                arm_model,
                " backend:=mock",
                " cable_kinematics_file:=",
                cable_kinematics_file,
            ]
        )
    }
    gui_description = {
        "robot_description": Command(
            [
                FindExecutable(name="python3"),
                " ",
                gui_generator,
                " ",
                cable_kinematics_file,
            ]
        )
    }
    return LaunchDescription(
        [
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
                "output_shaft_topic",
                default_value="/pi_arm/model_output_shaft_states",
                description="JointState topic carrying six motor output-shaft angles in rad",
            ),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="pi_arm_output_visualizer_state_publisher",
                output="screen",
                parameters=[arm_description],
            ),
            Node(
                package="pi_arm_output_visualizer",
                executable="output_shaft_visualizer_node",
                output="screen",
                parameters=[
                    {
                        "cable_kinematics_file": cable_kinematics_file,
                        "input_topic": output_shaft_topic,
                        "output_topic": "/joint_states",
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="pi_arm_output_shaft_description_publisher",
                output="screen",
                parameters=[gui_description],
                remappings=[
                    ("/robot_description", "/pi_arm/output_shaft_robot_description"),
                    ("/joint_states", output_shaft_topic),
                    ("/tf", "/pi_arm/output_shaft_tf"),
                    ("/tf_static", "/pi_arm/output_shaft_tf_static"),
                ],
                condition=IfCondition(use_gui),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="pi_arm_output_shaft_gui",
                output="screen",
                # The isolated description topic is generated from the same
                # YAML used by the converter and hardware backend, so slider
                # ranges cannot drift from the enforced output-shaft limits.
                remappings=[
                    ("/joint_states", output_shaft_topic),
                    ("/robot_description", "/pi_arm/output_shaft_robot_description"),
                ],
                condition=IfCondition(use_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
