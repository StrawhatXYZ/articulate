# Source of the SO-101 model files

The meshes in this directory and `../urdf/upstream/so101_new_calib.urdf` are
copied unmodified from TheRobotStudio's SO-ARM100 repository:

- Repository: <https://github.com/TheRobotStudio/SO-ARM100>
- Path: `Simulation/SO101/` (`assets/*.stl`, `so101_new_calib.urdf`)
- Commit: `a758567c3978dfeefe282ede0500085a48fe8f78` (6 October 2026)
- Licence: Apache License 2.0 (`LICENSE` in this directory)

Only the meshes used by the follower arm are included.

`../urdf/so101.urdf.xacro` is generated from the upstream URDF by
`../tools/generate_xacro.py`. Its kinematics, inertials, joint limits and
visual meshes are the upstream values. It changes the following:

- Collision geometry: boxes derived from the mesh bounds instead of meshes, plus box
  pads on the jaws. The moving pad is tilted so that it closes flat on a 3 cm
  piece.
- Effort and velocity limits: 2.94 N m and 4.7 rad/s (STS3215 servo), not the
  placeholder 10/10. Joint damping and friction come from the upstream MuJoCo
  motor model.
- Mesh paths are absolute (`file://$(find so101_description)/meshes/...`).
- ROS 1 transmissions are removed.
- Industrial colours replace the upstream yellow.
- A `grasp_link` frame is added between the open jaws.
