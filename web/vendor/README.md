# Local 3D dependencies

The console loads its 3D display libraries from the committed `viewer-libs.js`.
Source imports are in `entry.js`.

- Three.js 0.180.0, including OrbitControls, STLLoader and URDF Loader's Collada dependency.
  Upstream: <https://github.com/mrdoob/three.js/tree/r180>. MIT, see `licenses/three.txt`.
- URDF Loader 0.12.6. Upstream: <https://github.com/gkjohnson/urdf-loaders>.
  Apache-2.0, see `licenses/urdf-loader.txt` (including Caltech attribution).
- esbuild 0.25.10 produces the bundle during maintenance.

`dependencies.json` pins release tarballs and SHA-512 integrity values. To reproduce
the bundle on Linux arm64 or x64 (maintainer task; requires Internet access):

```bash
python3 scripts/build_web_vendor.py
```

The builder downloads into a temporary directory, verifies checksums and produces
the bundle. Distribute the license files and `viewer-libs.js.LEGAL.txt` with it.
