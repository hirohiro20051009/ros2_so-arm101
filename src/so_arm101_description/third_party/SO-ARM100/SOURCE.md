# Source of the files in this directory

These files are copies of upstream files and are **not modified**.

| item | value |
|---|---|
| upstream | https://github.com/TheRobotStudio/SO-ARM100 |
| file | `Simulation/SO101/so101_new_calib.urdf` |
| permalink | https://github.com/TheRobotStudio/SO-ARM100/blob/385e8d7c68e24945df6c60d9bd68837a4b7411ae/Simulation/SO101/so101_new_calib.urdf |
| commit of the file (last change) | `385e8d7c68e24945df6c60d9bd68837a4b7411ae` (2025-07-02, "fix(urdf): fixing multiple issues related to the last URDF update (#117)") |
| upstream HEAD when fetched | `a758567c3978dfeefe282ede0500085a48fe8f78` (2026-10-06). The file is the same at that HEAD |
| git blob of the file | `9552a231d8b23bed68ec15779eba620c5d875ec4` |
| license | Apache License 2.0 (`LICENSE` here is a copy of upstream `LICENSE` at the same commit; no NOTICE file upstream) |
| fetched | 2026-11-12, with `git show 385e8d7:<path>` (the commit is pinned; the working tree was not used) |

## Files

- `so101_new_calib.urdf`: upstream file, byte for byte.
- `LICENSE`: upstream license text, byte for byte.
- `../../meshes/*.stl` (13 files): copies of upstream `Simulation/SO101/assets/*.stl` at the same commit
  (identical sha256 at `385e8d7` and at the HEAD above). They are kept in `meshes/` (not copied again) and are
  listed in `SHA256SUMS` so that any change is detected by the test `urdf_original_checksums`.
  Upstream has 5 more STL files (leader arm and camera parts) that this URDF does not use; they are not copied.
- `SHA256SUMS`: `sha256sum` format, paths relative to the package root (`src/so_arm101_description`).

## How it is used

The URDF is not installed as it is. `CMakeLists.txt` generates `build/.../urdf/so101_new_calib.urdf` with
`cmake/gen_urdf.cmake`; the only change is the mesh path prefix
`filename="assets/` -> `filename="package://so_arm101_description/meshes/` (34 places), plus one XML comment line that
names the source. The generated file is installed to `share/so_arm101_description/urdf/so101_new_calib.urdf`.
`cmake/gen_urdf.cmake` stops the build if the sha256 of this file differs from `SHA256SUMS` or if the number of
`assets/` references is not 34. The `<transmission>` elements of the upstream file are kept (ros2_control ignores
`<transmission>` outside `<ros2_control>`; see `docs/original_urdf_plan.md`).

Notes on upstream's own comments: the URDF was generated with onshape-to-robot from an Onshape document (URLs are in
the first lines of the file); upstream's README says the motor properties come from the Open Duck Mini project.
