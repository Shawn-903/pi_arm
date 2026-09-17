# Local 3D dependencies

The console loads the committed `viewer-libs.js`; no CDN, npm, Node.js, or Internet
connection is required at runtime. Source imports are in `entry.js`.

- Three.js 0.180.0, including OrbitControls, STLLoader and URDF Loader's Collada dependency.
  Upstream: <https://github.com/mrdoob/three.js/tree/r180>. MIT, see `licenses/three.txt`.
- URDF Loader 0.12.6. Upstream: <https://github.com/gkjohnson/urdf-loaders>.
  Apache-2.0, see `licenses/urdf-loader.txt` (including Caltech attribution).
- esbuild 0.25.10 is used only to produce the bundle, not shipped as an executable.

`dependencies.json` pins release tarballs and SHA-512 integrity values. To reproduce
the bundle on Linux arm64 or x64 (maintainer task; requires Internet access):

```bash
python3 scripts/build_web_vendor.py
```

The builder downloads into a temporary directory, verifies checksums and bundles
without npm install hooks. Keep license files and `viewer-libs.js.LEGAL.txt` with the
bundle. These dependencies do not open robot connections or implement motor control.
