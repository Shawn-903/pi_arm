#!/usr/bin/env python3
"""Export display assets from the ROS model; never opens CAN or starts ROS nodes."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "src/pi_arm_description"


def export(output):
    import xacro

    source = DESCRIPTION / "urdf/pi_arm.urdf.xacro"
    robot = ET.fromstring(xacro.process_file(str(source)).toxml())
    controls = robot.find("ros2_control")
    names = [joint.attrib["name"] for joint in controls.findall("joint")]
    robot.remove(controls)
    # Visual-only export. The same joint transforms/limits/mimics are retained.
    for link in robot.findall("link"):
        for element in list(link):
            if element.tag != "visual":
                link.remove(element)
    output.mkdir(parents=True, exist_ok=True)
    meshes = output / "meshes"
    meshes.mkdir(exist_ok=True)
    records = []
    for mesh in robot.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        prefix = "package://pi_arm_description/"
        if not filename.startswith(prefix):
            raise ValueError(f"Unexpected mesh package: {filename}")
        path = (DESCRIPTION / filename[len(prefix):]).resolve()
        path.relative_to(DESCRIPTION.resolve())
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        # Content-addressed immutable mesh URLs prevent stale browser caches.
        dest = meshes / f"{path.stem}-{digest[:16]}.stl.gz"
        if not dest.exists():
            dest.write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))
        mesh.set("filename", "meshes/" + dest.name)
        records.append(dict(source=str(path.relative_to(ROOT)), sha256=digest,
                            url="meshes/" + dest.name, source_bytes=len(raw),
                            compressed_bytes=dest.stat().st_size))
    xml = ET.tostring(robot, encoding="unicode", xml_declaration=True)
    (output / "pi_arm.urdf").write_text(xml + "\n")
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    manifest = dict(schema=1, source=str(source.relative_to(ROOT)),
                    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    urdf_sha256=hashlib.sha256((xml + "\n").encode()).hexdigest(),
                    urdf="pi_arm.urdf", root="base_link", tip="tool0",
                    joint_names=names,
                    limits_rad={name: {key: float(joints[name].find("limit").attrib[key])
                                       for key in ("lower", "upper")} for name in names},
                    meshes=records)
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Web model: {len(names)} axes, {len(records)} meshes, "
          f"{sum(r['compressed_bytes'] for r in records) / 1024**2:.1f} MiB compressed")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "web/model")
    export(parser.parse_args().output)
