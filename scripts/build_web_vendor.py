#!/usr/bin/env python3
"""Optional maintainer build; normal startup uses the committed local JS bundle."""
import base64
import hashlib
import io
import json
from pathlib import Path
import platform
import subprocess
import tarfile
import tempfile
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "web/vendor"


def main():
    arch = {"aarch64": "arm64", "x86_64": "x64"}.get(platform.machine())
    if platform.system() != "Linux" or not arch:
        raise SystemExit("Maintainer build supports Linux arm64/x64")
    packages = json.loads((VENDOR / "dependencies.json").read_text())
    with tempfile.TemporaryDirectory(prefix="pi-arm-viewer-") as temp:
        root = Path(temp)
        for name in ["three", "urdf-loader", f"@esbuild/linux-{arch}"]:
            entry = packages[name]
            with urlopen(entry["url"], timeout=120) as response:
                data = response.read()
            actual = "sha512-" + base64.b64encode(hashlib.sha512(data).digest()).decode()
            if actual != entry["integrity"]:
                raise ValueError(f"Dependency checksum failed: {name}")
            with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
                for member in archive.getmembers():
                    if not member.isfile():
                        continue
                    rel = Path(member.name).relative_to("package")
                    if ".." in rel.parts:
                        raise ValueError("Invalid archive path")
                    target = root / "node_modules" / name / rel
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(archive.extractfile(member).read())
        binary = root / f"node_modules/@esbuild/linux-{arch}/bin/esbuild"
        binary.chmod(0o755)
        (root / "entry.js").write_bytes((VENDOR / "entry.js").read_bytes())
        subprocess.run([str(binary), "entry.js", "--bundle", "--minify",
                        "--format=iife", "--global-name=PiArm3DLibs",
                        "--target=es2020", "--legal-comments=external",
                        "--outfile=" + str(VENDOR / "viewer-libs.js")], cwd=root, check=True)


if __name__ == "__main__":
    main()
