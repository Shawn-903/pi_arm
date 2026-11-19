from pathlib import Path

import pytest

from pi_arm_tasks.zero_reference import files


@pytest.fixture
def paths(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setattr(files, "get_package_share_directory", lambda _: str(tmp_path))
    zero = tmp_path / ".local/share/pi_arm/software-zero-vcan0.yaml"
    zero.parent.mkdir(parents=True)
    return tmp_path / "config", zero


def test_first_capture_default_and_mock_isolation(paths):
    config, zero = paths
    template = config / "pack_reference_calibration.template.yaml"
    assert files.resolve_reference_files("vcan0") == (zero, zero)
    assert files.resolve_reference_files("vcan0", capture=True) == (zero, zero)
    zero.write_text("calibration: {}\nsoftware_zero: {schema_version: 2}\n")
    assert files.resolve_reference_files("vcan0") == (zero, zero)
    assert files.resolve_reference_files("vcan0", mock=True) == (template, None)
    other = zero.with_name("software-zero-vcan1.yaml")
    assert files.resolve_reference_files("vcan1") == (other, other)


def test_legacy_custom_paths_and_corrupt_file(paths):
    config, zero = paths
    factory = config / "pack_reference_calibration.R10.recorded.yaml"
    custom = Path(str(zero) + ".custom")
    zero.write_text("software_zero: {schema_version: 1}\n")
    assert files.resolve_reference_files("vcan0") == (factory, zero)
    assert files.resolve_reference_files("vcan0", capture=True) == (zero, zero)
    assert files.resolve_reference_files("vcan0", custom) == (custom, zero)
    zero.write_text("software_zero: [")
    assert files.resolve_reference_files("vcan0") == (zero, zero)
    custom.write_text("calibration: {}\nsoftware_zero: {schema_version: 2}\n")
    assert files.resolve_reference_files("vcan0", custom) == (custom, custom)
    assert files.resolve_reference_files("vcan0", software_zero_file=custom) == (custom, custom)
