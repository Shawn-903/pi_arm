from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _launch_setup(context):
    urdf_path = (
        get_package_share_directory("pi_arm_description")
        + "/urdf/pi_arm.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("pi_arm", package_name="pi_arm_moveit_config")
        .robot_description(
            file_path=urdf_path,
            mappings={
                "backend": LaunchConfiguration("hardware_type").perform(context),
                "can_interface": LaunchConfiguration("can_interface").perform(context),
                "bitrate": LaunchConfiguration("bitrate").perform(context),
                "cable_kinematics_file": LaunchConfiguration(
                    "cable_kinematics_file"
                ).perform(context),
                "calibration_file": LaunchConfiguration(
                    "calibration_file"
                ).perform(context),
                "allow_real_control": LaunchConfiguration(
                    "allow_real_control"
                ).perform(context),
            },
        )
        .robot_description_semantic(file_path="config/pi_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    generated = generate_move_group_launch(moveit_config)
    moveit_package = Path(get_package_share_directory("moveit_ros_move_group")) / "package.xml"
    if ET.parse(moveit_package).getroot().findtext("version") == "2.12.4":
        # Callback groups can outlive these plugin loaders in MoveIt 2.12.4.
        # Keep their library code mapped until process exit (see this package's README).
        libraries = [
            Path(get_package_prefix("moveit_simple_controller_manager"))
            / "lib/libmoveit_simple_controller_manager.so",
            Path(get_package_prefix("moveit_ros_move_group"))
            / "lib/libmoveit_move_group_default_capabilities.so",
        ]
        for library in libraries:
            if not library.is_file():
                raise RuntimeError(f"MoveIt 2.12.4 shutdown workaround requires {library}")
        preload = " ".join([*(str(path) for path in libraries),
                            context.environment.get("LD_PRELOAD", "")]).strip()
        # Scoped to this move_group launch; preserve any caller-supplied preload.
        return [GroupAction(actions=[
            SetEnvironmentVariable("LD_PRELOAD", preload),
            *generated.entities,
        ])]
    return list(generated.entities)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_type", default_value="mock", choices=["mock", "real"]
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("bitrate", default_value="1000000"),
            DeclareLaunchArgument(
                "cable_kinematics_file",
                default_value=(
                    get_package_share_directory("pi_arm_description")
                    + "/config/cable_kinematics.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=(
                    get_package_share_directory("pi_arm_hardware")
                    + "/config/pack_reference_calibration.template.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "allow_real_control", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
