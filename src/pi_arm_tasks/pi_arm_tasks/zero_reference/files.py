"""选择当前 CAN 接口的零点文件与首次采集使用的基础配置。"""

import os
from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
import yaml


def _document(path):
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError):
        # 硬件加载器报告原始文件错误；只有明确识别的旧文件才使用外部基础配置。
        return {}
    return value if isinstance(value, dict) else {}


def resolve_reference_files(can_interface, calibration_file=None, software_zero_file=None,
                            *, capture=False, mock=False):
    config = Path(get_package_share_directory("pi_arm_hardware")) / "config"
    template = config / "pack_reference_calibration.template.yaml"
    factory = config / "pack_reference_calibration.R10.recorded.yaml"
    base = Path(calibration_file).expanduser().absolute() if calibration_file else None
    if mock:
        return base or template, None

    if software_zero_file:
        zero = Path(software_zero_file).expanduser().absolute()
    elif base and "software_zero" in _document(base):
        zero = base
    else:
        zero = Path.home() / ".local/share/pi_arm" / f"software-zero-{can_interface}.yaml"

    if os.path.lexists(zero):
        if base is None:
            document = _document(zero)
            record = document.get("software_zero")
            legacy = ("calibration" not in document and isinstance(record, dict)
                      and record.get("schema_version") == 1)
            base = factory if legacy else zero
        return base, zero
    if capture or software_zero_file:
        return base or factory, zero
    return base or template, None


def configure_launch_references(context, *, monitor=False):
    args = context.launch_configurations
    base, zero = resolve_reference_files(
        args["can_interface"], args["calibration_file"], args["software_zero_file"],
        capture=monitor and args["commissioning"] == "true",
        mock=not monitor and args["hardware_type"] == "mock")
    args["calibration_file"] = str(base)
    args["software_zero_file"] = str(zero) if zero is not None else ""
    args["reference_xacro_args"] = shlex.join([
        "calibration_file:=" + args["calibration_file"],
        "software_zero_file:=" + args["software_zero_file"],
    ])
