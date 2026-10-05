# Step 1: Native Image, Pick-and-Place and Benchmark Harness

- **Status:** Draft, awaiting review
- **Date:** 5 October 2026
- **Roadmap position:** Step 1 of 5 (milestone M1)

## 1. Purpose

The lab is simulation-only until results justify buying hardware (target: a
low-cost SO-101 arm, later a LEAP Hand). Every later step must therefore show
measurable improvement against a fixed reference. Step 1 provides:

1. a **native arm64 image**, so the lab runs without emulation on Apple Silicon;
2. a **visible pick-and-place capability** on the current 3-DOF arm, using a
   simulated suction gripper;
3. a **benchmark harness** that runs randomized, seeded trials and reports
   success, speed and precision with honest statistics;
4. a committed **baseline** that all later "10x" claims are measured against.

## 2. Decisions

| Topic | Decision | Rejected alternative |
|---|---|---|
| Base image | Multi-arch `ros:jazzy`, pinned by index digest, plus `ros-jazzy-desktop` | Per-architecture base images (two code paths) |
| Trial execution | One long-running headless simulation, scene reset between trials | Fresh simulation per trial (~15–20 s start-up each); `launch_testing` only |
| Grasping | Suction via Gazebo `DetachableJoint`, gated by a measured proximity check | Unconditional attach (would make success rates meaningless) |
| Inverse kinematics | Closed-form for the 3-DOF arm | Motion planner (MoveIt is step 2) |
| Object visualisation | RViz markers from live Gazebo poses | Relying on the Gazebo GUI rendering |

Verified before this spec: all required packages (`ros-jazzy-desktop`,
`ros-jazzy-ros-gz`, `ros-jazzy-moveit`, `ros-jazzy-ros2-control`,
`ros-jazzy-ros2-controllers`, `ros-jazzy-gz-ros2-control`, the noVNC desktop
stack) exist for arm64, and `libgz-sim8-detachable-joint-system` ships with
Gazebo 8.

## 3. Components

### 3.1 Image

- `Dockerfile`: `FROM ros:jazzy@sha256:…`, using the multi-architecture
  index digest reported by `docker buildx imagetools inspect ros:jazzy` at
  implementation time (not a single-platform manifest digest); install
  `ros-jazzy-desktop` together with the existing package list. Layer order,
  workspace build and desktop setup are otherwise unchanged.
- `docker-compose.yml`: remove `platform: linux/amd64`.
- Before the switch, the current image is re-tagged
  `ai-robotics-lab:amd64-emulated` so both can be benchmarked (Section 6).

### 3.2 Scene

- `worlds/table_world.sdf`: the three pieces are moved inside the arm's
  reachable region (Section 4.2). Their sizes, masses and colours are
  unchanged.
- Pose publishing: each piece model carries a `PosePublisher` system that
  publishes its world pose on `/model/<piece>/pose`. (Verified in the step 1
  probe: Gazebo's combined `dynamic_pose/info` stream loses entity names when
  bridged to `TFMessage`, so it cannot identify pieces.)
- Joint limits: `joint_2` (shoulder) and `joint_3` (elbow) widen from ±1.5708
  to **±2.356 rad (±135°)** in `simple_arm.urdf`, `simple_arm.sdf` and
  `move_arm.py`'s `JOINT_LIMITS`. With ±90° the table is unreachable
  (Section 4.2). Link lengths, masses and the demo poses are unchanged.
- Suction: one `DetachableJoint` per piece, parent `end_effector`, child the
  piece's link, each with its own attach and detach topics
  (`/simple_arm/suction/<piece>/attach` and `.../detach`). The plugins are
  declared inside the world file's `<include>` of the arm, not in
  `simple_arm.sdf`, so the arm model does not name this world's pieces.

### 3.3 Bridge (`config/bridge.yaml`)

| ROS | Gazebo | Direction | Purpose |
|---|---|---|---|
| `/lab/pieces/<piece>/pose` (`geometry_msgs/msg/PoseStamped`) | `/model/<piece>/pose` (`gz.msgs.Pose`) | GZ→ROS | Live pose of each piece |
| `/simple_arm/suction/<piece>/attach` (`std_msgs/msg/Empty`) | same (`gz.msgs.Empty`) | ROS→GZ | Engage suction |
| `/simple_arm/suction/<piece>/detach` (`std_msgs/msg/Empty`) | same (`gz.msgs.Empty`) | ROS→GZ | Release suction |
| `/world/table_world/set_pose` (`ros_gz_interfaces/srv/SetEntityPose`) | `/world/table_world/set_pose` | service | Scene reset |

### 3.4 Python package `lab_simulation`

Installed with `ament_cmake_python` (`ament_python_install_package`). Each
module has one responsibility and a narrow interface.

| Module | Responsibility | Interface (summary) | Depends on |
|---|---|---|---|
| `kinematics.py` | Closed-form FK/IK for the 3-DOF arm | `forward(q) -> xyz`; `inverse(xyz) -> q`, raises `Unreachable` | Constants mirrored from the URDF |
| `arm.py` | Command joints and wait for settling | `move_to(q, timeout) -> MoveResult`; `joint_positions()` | ROS topics |
| `scene.py` | Layouts, reset, piece poses, suction | `sample_layout(rng)`; `reset(layout)`; `pose(piece)`; `attach(piece) -> bool`; `detach(piece)` | Bridge |
| `tasks.py` | Task definitions | `ReachTask`, `PickPlaceTask`; `run(rng) -> TrialResult` | `kinematics`, `arm`, `scene` |
| `metrics.py` | Aggregation and statistics | Wilson interval, percentiles, summary | none |
| `benchmark.py` | CLI: run, report, compare | `run ...`, `compare A B` | `tasks`, `metrics` |
| `object_markers.py` | RViz markers for the pieces | Node publishing `visualization_msgs/msg/MarkerArray` | Bridge |

`kinematics.py` and `metrics.py` contain no ROS imports, so they are
unit-testable without a simulation.

### 3.5 Launch

- `arm_world.launch.py`: also starts `object_markers`; the RViz config gains a
  MarkerArray display.
- `benchmark.launch.py` (new): headless Gazebo, bridge, then
  `benchmark run` with arguments `task` (`reach` | `pick_place` | `all`),
  `trials`, `seed`, `out`, and `rviz` (default `false`).

`move_arm.py` is unchanged.

## 4. Trial Behaviour

### 4.1 Reset (before every trial)

1. Detach all suction joints.
2. Move the arm home and wait until settled.
3. Place the pieces at a seeded random layout via `set_pose`.
4. Wait until every piece has a linear speed below 1 mm/s (timeout 3 s).

### 4.2 Reachable region

Pieces and targets are sampled only where IK succeeds, within joint limits,
for both the grasp height and the hover height, with a 1 cm margin in every
direction. The shoulder sits 0.29 m above the table, so reaching the table
requires the shoulder to lean past horizontal: with ±90° limits no table
position is reachable at all. With the ±135° limits of Section 3.2, the ring
common to all three pieces is 0.12–0.27 m from the base axis (computed with
the closed-form IK). The sampler derives this from `kinematics.py` and does
not hard-code it; it also keeps pieces at least 0.12 m from the base axis
(the base radius is 0.08 m) and at least 8 cm apart, centre to centre.

### 4.3 Reach task

- Target: a seeded random reachable point at least 5 cm above the table.
- Success: tip position error ≤ 1 cm after settling, measured by applying FK
  to the measured joint positions.
- Recorded: tip error, settle time, per-joint tracking error.

### 4.4 Pick-and-place task

Sequence: hover 5 cm above the chosen piece, descend to contact height,
proximity check, attach, lift 5 cm, move above the target, lower, detach,
retreat home, wait for the piece to settle.

- **Proximity check:** the gap between the tip sphere's surface and the
  piece's top surface (from live Gazebo poses) must be ≤ 5 mm, or the trial
  fails with `grasp_miss` and `attach` is never sent.
- **Carry check:** during the carry, the piece must stay within 3 cm of its
  attached offset from the tip; otherwise `dropped`. (Amended from 1 cm with
  the user's approval: a held piece hangs ~1.1 cm off its grip because the
  suction joint is slightly compliant, so 1 cm labelled every held piece as
  dropped. Success still requires placement within 2 cm.)
- **Success:** grasp passed, no drop, and final piece position within 2 cm
  (in the table plane) of the target.
- **Recorded:** cycle time; placement error; whether other pieces moved more
  than 1 cm (`disturbed`, reported but not a failure).

### 4.5 Failures and robustness

- Failure reasons: `unreachable`, `grasp_miss`, `dropped`,
  `placement_error`, `timeout`.
- Each step has a timeout. A timeout fails the current trial; the runner
  resets and continues.
- If `/joint_states` is silent for 5 s, the run aborts with a clear error.
- Each trial is appended to `trials.jsonl` as it completes, so interrupted
  runs keep their data.

### 4.6 Timing and simulation speed

- The world file caps simulation at real time (`real_time_factor` 1.0), which
  would hide any machine speed-up. The benchmark lifts the cap at start-up
  through the `/world/table_world/set_physics` service with
  `real_time_factor: 1000` (measured: `0` leaves the cap in place; `100`
  reached 9.8× real time on the emulated image). Interactive launches keep
  the real-time cap.
- Robot timings (cycle time, settle time, step timeouts) are measured in
  **simulation seconds**: they describe the robot and are identical on any
  machine.
- Throughput (trials per minute, achieved real-time factor) is measured in
  **wall-clock time**: it describes the machine and image.
- A separate wall-clock watchdog detects an unresponsive simulation (Section
  4.5).

### 4.7 Reproducibility

The seed fully determines layouts, piece choice and targets. Physics may vary
slightly between runs, so comparisons are made on aggregate rates, never on
single trials.

## 5. Report and Comparison

### 5.1 Output (`results/<YYYYMMDD-HHMMSS>/`)

- `trials.jsonl`: one record per trial (task, index, outcome, failure reason,
  metrics, timings).
- `summary.json`: aggregates plus provenance: git commit and dirty flag,
  CPU architecture, ROS distro, Gazebo version, seed, trial count, date.
- `report.md`: human-readable tables.

### 5.2 Metrics

| Metric | Statistic |
|---|---|
| Success rate per task | Rate with 95% Wilson interval |
| Cycle time | Mean, median, p95 |
| Tip error / placement error | Mean, p95 |
| Failure reasons | Count per reason |
| Simulation speed | Mean real-time factor; trials per minute |

### 5.3 Comparison

`benchmark compare <A> <B>` prints every metric side by side with the absolute
change and the ratio. A success-rate change whose intervals overlap is marked
**not significant**.

### 5.4 Storage

- `results/` is git-ignored.
- Milestone runs copy `summary.json` and `report.md` into
  `benchmarks/<milestone>/`, which is committed. Step 1 creates
  `benchmarks/baseline/`.

## 6. Testing

### 6.1 Unit tests (no simulation; `colcon test` and CI)

- Kinematics: FK/IK round-trip over a grid; unreachable targets raise;
  results respect joint limits; constants match the URDF.
- Layout sampler: seed determinism, separation, reachability.
- Metrics: Wilson interval and percentiles against known values.
- Report/compare: fixture inputs produce the expected deltas and
  significance flags.
- `test_robot_description.py` extended: every `DetachableJoint` references an
  existing link and model; every new bridge entry matches its Gazebo topic.

### 6.2 Simulation tests

- Local: `benchmark.launch.py trials:=5` as a smoke test.
- CI, new job: headless 10-trial benchmark on every push; the report is
  uploaded as a run artifact.
- CI, new job: Docker image build, triggered only by changes to `Dockerfile`,
  `docker-compose.yml` or `scripts/`.

## 7. Done Criteria

1. The image builds and runs natively on arm64 (container reports `aarch64`,
   no platform warning) and builds for amd64 in CI.
2. Existing behaviour is preserved: the 32 existing tests and all new tests
   pass, `move_arm.py` exits 0, RViz works.
3. Pick-and-place is visible live in RViz through noVNC, with the pieces
   rendered.
4. A baseline of 100 trials per task (reach and pick-and-place, seed 42) is
   committed in `benchmarks/baseline/`.
5. The baseline includes a measured native-versus-emulated comparison, run
   with the same seed on `ai-robotics-lab:amd64-emulated` and on the native
   image. The comparison covers only the tasks the old image can run.
6. The Gazebo GUI is tested on the native image, and the README states the
   result.
7. The README documents the benchmark, the pick demo and how to read the
   report.

## 8. Risks and Fallbacks

| Risk | Detection | Fallback |
|---|---|---|
| `DetachableJoint` creates its joint at start-up (attached) | First implementation task is a probe | Detach all at start-up and re-attach on command if `attach_topic` is supported; otherwise carry kinematically (piece pose follows the tip via `set_pose` each step) with the same proximity and carry checks |
| `set_pose` service not bridgeable in this ros_gz version | Same probe | Call `gz service` from the reset routine |
| Reachable ring (0.12–0.27 m) too tight for three pieces plus targets | Sampler unit tests (layouts must succeed for many seeds) | Reduce pieces per layout, or lengthen links in both descriptions (consistency test enforces both) |
| Gazebo GUI still blank natively | Done criterion 6 | RViz remains the supported viewer; document it |
| Native `ros-jazzy-desktop` build changes rendering or timing | Existing tests plus a `move_arm.py` run on the new image | Retune gains only if poses fail; record the change |

## 9. Out of Scope

SO-101 model, MoveIt integration, real grippers, cameras and perception,
voice commands, webcam hand-tracking teleoperation, learned policies. These
belong to steps 2–5 and to the separate voice/webcam sub-project.
