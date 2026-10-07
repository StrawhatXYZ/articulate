# Step 2: SO-101 Arm, Two-Finger Grasping, ros2_control and MoveIt

- **Status:** Approved for implementation (user delegated all design decisions)
- **Date:** 7 October 2026
- **Roadmap position:** Step 2 of 5 (milestone M2)

## 1. Purpose

Step 1 measured pick-and-place on an invented 3-DOF arm holding pieces by
suction. A funder would discount both: it is not a real robot, and suction in
simulation glues the piece to the tip. Step 2 moves every number onto the
robot the lab would buy, grasping the way that robot grasps:

1. the **SO-101** arm (5 joints plus a one-jaw gripper), from its official
   open-source model;
2. **ros2_control** drives the arm and gripper in Gazebo through the same
   controller interfaces a real SO-101 driver exposes;
3. **MoveIt 2** plans collision-aware motions for every move;
4. the gripper **closes its jaws on the piece**; no proximity-triggered suction;
5. the step 1 benchmark (tasks, seed, statistics) runs on the SO-101, with a
   committed 100-trial baseline and a comparison report against step 1.

Out of scope: cameras, voice, learned policies, the LEAP Hand, real hardware.

## 2. Findings from the feasibility spike

Measured in this lab before the design was fixed (7 October 2026):

| Finding | Consequence |
|---|---|
| The official URDF (TheRobotStudio/SO-ARM100, Apache-2.0) loads in Gazebo Harmonic and RViz. | Use it unmodified for kinematics, inertia and limits; vendor its meshes. |
| gz_ros2_control's position interface defaults to `position_proportional_gain` 0.1, so a joint takes ~10 s to settle. | Set the gain to 10 and the controller rate to the 1 kHz physics rate: tracking error < 0.005 rad. |
| Mesh collisions cut the real-time factor to ~0.6x idle and ~1x while holding. | Collision geometry is primitive boxes: ~5x real time while holding. |
| A gripper action controller stops squeezing once it detects a stall. | The gripper is a plain position controller: it keeps commanding "closed", so the servo squeezes at its torque limit like a real STS3215. |
| The moving jaw swings in an arc; a pad parallel to the fixed jaw meets the piece on an edge and the piece squirts out. | The moving pad is tilted -0.25 rad so it lands flat on a 3 cm piece. |
| Even then, Gazebo (DART and Bullet engines alike) cannot hold a friction grasp indefinitely: the piece creeps out within 10-40 s of simulated time. Lower torque or torque control makes it worse. | A **grasp latch** (section 4.4), disclosed in every report, plus a friction-only mode that is benchmarked once for the record. |
| With a vertical approach the SO-101 reaches grasp height from 0.08 to 0.28 m from its pan axis and a 6 cm hover from 0.10 to 0.24 m. | Pieces are laid out 0.12-0.22 m from the pan axis within +/-100 degrees of straight ahead. |

## 3. Decisions

| Topic | Decision | Rejected alternative |
|---|---|---|
| Robot model | Official SO-101 URDF ("new calibration"), meshes vendored with their licence | Hand-built approximation; LeRobot's MuJoCo model |
| One source of truth | URDF (xacro) is spawned into Gazebo from `robot_description`; no separate SDF model | Parallel SDF and URDF kept identical by tests (step 1) |
| Collision geometry | Boxes generated once from the mesh bounds; box jaw pads | Meshes (5-8x slower), convex hulls |
| Arm control | `joint_trajectory_controller` on position interfaces | Effort control with PID (needs per-joint tuning, no gain) |
| Gripper control | `position_controllers/JointGroupPositionController` | Gripper action controllers (stop squeezing on stall) |
| Motion planning | `move_group` (OMPL RRTConnect), joint-space goals from our IK, driven through its action interface | `moveit_py` (heavy, in-process); MoveIt Task Constructor (step 3+) |
| IK for task goals | Numerical IK (NumPy) on the URDF chain with a vertical approach constraint | MoveIt pose goals (5-DOF arms make orientation goals ill-posed) |
| RViz IK plugin | `pick_ik` (handles under-actuated arms) | KDL (fails for 5-DOF orientation goals) |
| Grasp hold | Friction grasp, then a latch gated on a real jaw closure | Pure friction (piece creeps out); suction-style attach on proximity |
| Pieces | Resized to SO-101 scale: every piece is 3 cm across its grasp | Step 1 sizes (blue cylinder 6 cm wide, beyond the jaw design) |
| Step 1 arm | Retired; step 1 stays reproducible from tag `v0.2.0` | Maintaining two robots |

## 4. Design

### 4.1 Packages

```
src/
├── so101_description/          # robot model, reusable outside the lab
│   ├── urdf/so101.urdf.xacro   # macro: upstream kinematics + box collisions
│   ├── urdf/upstream/          # verbatim upstream URDF (reference for tests)
│   ├── meshes/                 # vendored STL + LICENSE + SOURCE.md (commit)
│   └── test/                   # parity with upstream, collision sanity
├── so101_moveit_config/        # SRDF, kinematics, limits, controllers, OMPL
│   └── launch/move_group.launch.py
└── lab_simulation/             # world, tasks, benchmark, demo
    ├── urdf/lab_so101.urdf.xacro   # robot + ros2_control + Gazebo plugins + latch
    └── config/ros2_controllers.yaml
```

`so101_description` knows nothing about Gazebo pieces; `lab_simulation`
wraps it with the simulation-only parts (ros2_control hardware plugin, grasp
latch, colours).

### 4.2 Robot model

- Kinematic chain, joint limits, inertials and visual meshes are copied from
  the upstream URDF verbatim; a test compares every joint origin, axis, limit
  and inertial against `urdf/upstream/so101_new_calib.urdf`.
- Effort limits are the STS3215's 2.94 Nm and velocity limits 4.7 rad/s
  (upstream lists placeholder 10/10); joint damping 0.6 and friction 0.052
  come from the upstream MuJoCo motor model.
- A fixed `grasp_link` sits between the open jaws, 22 mm from the fixed jaw's
  inner face and 4 mm behind its tip. Task goals target this link.
- Colours follow the lab's industrial scheme: printed parts orange
  (1.0 0.45 0.0), servos graphite (0.2 0.2 0.22).

### 4.3 Control

- Controllers at 1 kHz: `joint_state_broadcaster`, `arm_controller`
  (JointTrajectoryController, 5 arm joints), `gripper_controller`
  (JointGroupPositionController, `gripper`).
- `gz_ros_control.position_proportional_gain: 10.0`.
- Gripper positions: open 0.6 rad (~45 mm gap), closed command -0.17 rad
  (fully closed); a stall above 0.05 rad means something is between the jaws.

### 4.4 Grasping and the grasp latch

A grasp attempt: hover above the piece with the jaws open and aligned with the
piece's narrow axis, descend, command closed, wait for the jaw to stall. Then:

- **Miss** (`grasp_miss`): the jaw closed past 0.05 rad (nothing between the
  jaws), or the piece centre is more than 15 mm from `grasp_link`.
- **Grasp**: otherwise. In the default *latched* mode a Gazebo
  `DetachableJoint` locks the piece to `gripper_link` at its current pose, so
  the simulator's contact creep cannot shake it loose; the jaws keep
  squeezing. In *friction* mode nothing is added and the piece is held by
  friction alone.

The gate uses only what a real SO-101 can sense (gripper servo position)
plus the ground-truth check that the closed jaws hold *this* piece.
Reports state the mode in their header. The baseline is published for both
modes.

### 4.5 Motion

- `move_group` with OMPL RRTConnect, planning time 2 s, velocity and
  acceleration scaling 0.5, time-optimal parameterisation.
- The planning scene holds the table, every piece except the one being
  approached, and the held piece as an attached object while carrying.
- Each move is a joint-space goal from the numerical IK. Approach and
  retreat moves are short and planned the same way.
- Execution goes through `moveit_simple_controller_manager` to
  `/arm_controller/follow_joint_trajectory`.

### 4.6 Tasks and benchmark

The benchmark harness, statistics, CLI, exit codes and output format from
step 1 are unchanged. The tasks are reimplemented for the SO-101:

- **reach**: move `grasp_link`, pointing straight down, to a random reachable
  point 3-12 cm above the table; error is the FK position error from the
  measured joint states. Tolerance 1 cm (unchanged).
- **pick_place**: pick a random piece, place it at a random target. Success
  needs the piece within 2 cm of the target and upright on the table within
  1 cm (unchanged). Failures: `unreachable`, `planning_failed` (new),
  `timeout`, `grasp_miss`, `dropped`, `placement_error`.
- Run metadata gains `robot` and `grasp_mode`; a comparison across different
  robots or grasp modes carries a comparability warning, like seeds and
  trial counts already do.

### 4.7 Pieces

| Piece | Shape | Grasp width | Height | Mass |
|---|---|---|---|---|
| red_cube | 3 cm cube | 3 cm | 3 cm | 30 g |
| blue_cylinder | cylinder, 3 cm diameter | 3 cm | 5 cm | 35 g |
| green_block | 5 x 3 x 3 cm block | 3 cm | 3 cm | 45 g |

Laid out 0.12-0.22 m from the SO-101's pan axis, within +/-100 degrees,
at least 7 cm apart, upright with yaw 0.

## 5. Error handling

Unchanged from step 1 (simulation unresponsive -> exit 3; no simulation ->
exit 2). New: a planning failure or rejected goal is a trial failure
(`planning_failed`), never an abort; a gripper that does not settle within
3 s is `timeout`.

## 6. Testing

- `so101_description`: xacro expands; upstream parity; every link has
  collision boxes; jaw pads exist; effort/velocity limits as specified.
- `so101_moveit_config`: SRDF groups and end effector reference real links
  and joints; controllers in `moveit_controllers.yaml` match
  `ros2_controllers.yaml`; joint limits cover every arm joint.
- `lab_simulation`: FK against the upstream URDF chain; IK round trips and
  vertical approach; layout within the reach map; tasks with fakes (grasp
  gate, latch modes, failures); bridge, world and controller contracts.
- Simulation: `move_arm.py` (joint check through the arm controller) exits 0;
  a 5-trial benchmark in CI; the 100-trial baseline.

## 7. Success criteria

1. `ros2 launch lab_simulation arm_world.launch.py` shows the SO-101 in
   Gazebo and RViz with MoveIt's motion planning panel.
2. `benchmark.launch.py task:=pick_place trials:=5 rviz:=true realtime:=true`
   shows the SO-101 picking with its jaws.
3. A 100-trial baseline (latched) and a friction-only run are committed with a
   comparison against step 1.
4. All tests pass; CI green; README describes the SO-101, the latch and the
   numbers.
