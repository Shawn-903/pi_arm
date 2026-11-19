"""Load virtual-CAN test fixtures from the project's robot description."""

import io
from pathlib import Path
import runpy

import xacro

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "src/pi_arm_description"


def _description(cable, calibration, interface):
    return xacro.process_file(str(DESCRIPTION / "urdf/pi_arm.urdf.xacro"), mappings={
        "backend": "real", "can_interface": interface, "cable_kinematics_file": str(cable),
        "calibration_file": str(calibration),
    }).toxml()


def calibration_mapping(cable):
    tool = runpy.run_path(str(ROOT / "src/pi_arm_hardware/scripts/pi_arm_calibration_candidate"))
    xml = _description(cable, "", "vcan0")
    _, mapping = tool["parse_robot_description"](io.BytesIO(xml.encode("utf-8")))
    return mapping, tool["configuration_fingerprint"](cable, mapping)


def write_probe_description(directory, calibration, interface):
    path = directory / "probe.urdf"
    xml = _description(DESCRIPTION / "config/cable_kinematics.yaml", calibration, interface)
    path.write_text(xml, encoding="utf-8")
    return path
