# AI & Robotics Lab

[![CI](https://github.com/StrawhatXYZ/ai-robotics-lab/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/StrawhatXYZ/ai-robotics-lab/actions/workflows/ci.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

A containerised ROS 2 simulation lab for building an **instructed grasp**
system: a robot arm that picks up objects on spoken or written command,
developed in simulation first.

It simulates the **SO-101**, the low-cost open-source arm the lab plans to
buy, built from its official model. The arm picks pieces up with its own
two-finger gripper, driven through ros2_control, with every move planned by
MoveIt. Its headline task is colour sorting: putting each piece into the
box of its own colour. A seeded benchmark measures reach, pick-and-place and
sorting performance.
Everything runs inside Docker and is viewed in a web browser, so no host-side
ROS, X server or GPU is required.

| Component | Version | Role |
|---|---|---|
| ROS 2 | Jazzy | Middleware connecting perception, planning and control |
| Gazebo | Harmonic (gz-sim 8) | Physics simulation of the arm, table and objects |
| ros2_control | Jazzy | Arm and gripper controllers, running inside Gazebo (gz_ros2_control) |
| MoveIt 2 | Jazzy | Collision-aware motion planning for every move |
| RViz 2 | Jazzy | 3D visualisation of the robot model and TF |

## Requirements

- Docker Desktop with at least 8 GB of memory allocated
  (or Docker Engine on Linux)
- About 10 GB of free disk space for the image
- A web browser

| Host | Support |
|---|---|
| macOS, Apple Silicon | Verified. Runs natively (arm64). |
| macOS, Intel | Expected to work natively. |
| Linux, x86-64 | Expected to work natively. |
| Windows 10/11, x86-64 | Expected to work natively through WSL 2 (see [Windows](#windows)). |

The image is built on the multi-architecture `ros:jazzy` base, so it runs
natively on both arm64 and x86-64 hosts.

### Windows

The helper scripts are bash scripts, so run them from a **WSL 2** terminal
(for example Ubuntu from the Microsoft Store), not PowerShell or CMD:

1. Install Docker Desktop and enable *Use the WSL 2 based engine* and
   *WSL integration* for your distribution.
2. Clone the repository **inside the WSL file system** (for example
   `~/ai-robotics-lab`), not under `C:\` or `/mnt/c`. Bind mounts from the
   Windows file system are much slower.
3. Run `./start-lab.sh` and the other commands exactly as on macOS or Linux.
   <http://localhost:6080> opens in your normal Windows browser.

Without WSL, the same steps can be run directly from PowerShell:

```powershell
docker compose up -d --build
docker exec -it ai-robotics-lab bash    # then use the in-container commands below
```

The repository's `.gitattributes` forces LF line endings, so a Windows Git
checkout does not break the scripts that run inside the Linux container.

## Quick start

```bash
./start-lab.sh          # build the image, start the container, build the workspace
```

Open **<http://localhost:6080>** and click **Connect** to see the lab's
desktop. It stays plain black until the simulation is launched, so start it
next:

```bash
./enter-lab.sh
ros2 launch lab_simulation benchmark.launch.py task:=sort trials:=3 rviz:=true realtime:=true
```

RViz shows the SO-101 sorting the red cube, blue cylinder and green block
into the box of the same colour, at normal speed; each result is printed in
the terminal. Ctrl-C stops it early. Use `task:=pick_place` to watch it place
pieces on random spots instead.

To check plain joint motion instead, start the simulation and, in a second
terminal, run the six-pose joint check (it moves the arm and gripper through
the controllers; it does not plan or pick):

```bash
# Terminal 1                                  # Terminal 2
./enter-lab.sh                                ./enter-lab.sh
ros2 launch lab_simulation arm_world.launch.py   ros2 run lab_simulation move_arm.py
```

`move_arm.py` verifies each pose against `/joint_states` and exits with
status 0 only if all were reached.

```bash
./stop-lab.sh           # stop and remove the container (image and ./src are kept)
```

## Using the lab

### Launch arguments

```bash
ros2 launch lab_simulation arm_world.launch.py [argument:=value ...]
```

| Argument | Default | Description |
|---|---|---|
| `rviz` | `true` | Start RViz 2 with the arm, the pieces and MoveIt's motion planning panel |
| `gz_gui` | `false` | Also start the Gazebo GUI. It uses several CPU cores under software rendering, so RViz is the lighter viewer. |
| `use_sim_time` | `true` | Drive ROS nodes from the Gazebo `/clock` |

### Planning in RViz

In RViz's **MotionPlanning** panel, drag the arm's goal marker (or pick
*home* under *Goal State*) and press **Plan & Execute**; the arm moves in
Gazebo. The SO-101 has five arm joints, so not every gripper orientation
exists for a given point; the marker snaps to the nearest one the arm can
reach. Interactive planning knows the arm but not the table or the pieces
(the benchmark adds those to MoveIt's planning scene itself), so keep goals
above the table.

### Commanding joints directly

The arm takes trajectories, the gripper a position (radians; 0.6 is open,
-0.17 fully closed):

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll],
    points: [{positions: [0.5, -1.3, 0.6, 1.6, 0.0], time_from_start: {sec: 2}}]}}"
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
ros2 topic echo /joint_states
```

### Watching a pick

RViz shows the table and the pieces at their live positions. To watch the arm
pick and place pieces at normal speed, run inside the container:

```bash
ros2 launch lab_simulation benchmark.launch.py task:=sort trials:=3 rviz:=true realtime:=true
```

and open <http://localhost:6080>. `task:=pick_place` places pieces on random
spots instead. Without `realtime:=true` the benchmark runs
several times faster than real time.

### Running the tests

```bash
cd /workspace
colcon test --packages-select lab_simulation
colcon test-result --verbose
```

Use `colcon test` without `--packages-select` to test all three packages.
The suite runs the ROS 2 linters (flake8, pep257, xmllint, lint_cmake) and
the pytest files in each package's `test/`:

- `so101_description`: the generated model matches the upstream SO-101 URDF
  in every joint origin, axis, limit and inertial, and the generator's output
  is what is committed;
- `so101_moveit_config`: groups, named states and collision exclusions refer
  to real links and joints, and the controllers match ros2_control;
- `lab_simulation`: kinematics against the URDF, the workspace map against the
  IK, layout sampling, the planning-scene diffs, metrics, the benchmark
  runner, and the tasks against a fake robot (grasp gate, latch, drops,
  planning failures). `test_robot_description.py` ties the description to the
  controllers, the world's pieces, the bridge, the markers and `move_arm.py`.

### Continuous integration

`.github/workflows/ci.yml` runs on every push to `main` and on every pull
request:

- **Build and test** builds the three packages and runs the full test suite
  in the official `ros:jazzy` image.
- **Lint** runs ShellCheck on the shell scripts and Hadolint on the
  Dockerfile. Rules ignored on purpose are listed, with reasons, in
  `.hadolint.yaml`.
- **Benchmark smoke test** runs 5 trials per task in a headless simulation and
  uploads the report as the `benchmark-report` artifact.

`.github/workflows/docker.yml` builds the image whenever the `Dockerfile`, the
compose file, the scripts or the package change.

## Benchmark

`run-benchmark.sh` runs seeded trials against a headless simulation and writes
`results/<date-time>/` containing `trials.jsonl`, `summary.json` and
`report.md`.

```bash
./run-benchmark.sh                                  # all three tasks, 100 trials each, seed 42
TASK=sort TRIALS=10 ./run-benchmark.sh              # colour sorting only
TASK=pick_place TRIALS=10 ./run-benchmark.sh        # quick check
TASK=pick_place GRASP_MODE=friction ./run-benchmark.sh   # without the grasp latch
```

| Task | What happens | Success means |
|---|---|---|
| `reach` | The gripper, pointing down, moves to a random point 3–8 cm above the table, clear of the pieces | Grasp point within 1 cm |
| `pick_place` | The jaws close on a random piece; MoveIt carries it to a random target | Jaws closed on the piece, held within 1 cm of its grip while carried, placed within 2 cm of the target, upright (tilt under 15°) on the table |
| `sort` | Every piece, one after another, is put into the box of its colour | All three pieces end upright inside their own boxes (checked at the end, so a piece knocked out later counts) |

Failures are classified as `unreachable`, `planning_failed`, `grasp_miss`,
`dropped`, `placement_error` or `timeout`. Success rates carry 95% Wilson
intervals. Robot timings are in simulation seconds; trials per minute and
real-time factor are wall-clock. The benchmark lifts Gazebo's real-time cap
unless `realtime:=true`.

### Step 2 baseline (SO-101)

Seed 42, 100 trials per task, native arm64
([`benchmarks/step2/`](benchmarks/step2/), with the full comparison):

| | Step 1: 3-DOF arm, suction | Step 2: SO-101, jaws + latch | Step 2: SO-101, friction only |
|---|---|---|---|
| Reach success | 100% (95% CI 96–100%) | 100% (96–100%) | – |
| Reach error, mean | 2.4 mm | 0.4 mm | – |
| Reach time, mean | 3.5 s | 1.3 s | – |
| Pick-and-place success | 84% (76–90%) | **98% (93–99%)** | 73% (64–81%) |
| Placement error, mean | 10.5 mm | 6.6 mm | 6.5 mm |
| Pick-and-place cycle, mean | 15.6 s | 11.5 s | 11.2 s |
| Colour sorting success (all 3 pieces) | – | **93% (86–97%)** | – |
| Colour sorting time, mean | – | 37.6 s | – |

On the robot the lab would buy, picking with its own jaws and MoveIt planning
every move, pick-and-place success went from 84% to 98% (a significant
difference), cycles are 26% shorter and placements more accurate. The wrist
fixed step 1's main failure: pieces now arrive level instead of tilted.
Sorting fails mostly when the red cube starts where the arm cannot grip it
straight down (3 of 7 failures) or the tall cylinder tips over in its box (2).

Read the latched column with the latch in mind (see
[Grasping, and the grasp latch](#grasping-and-the-grasp-latch)): every grasp
was a real jaw closure, but Gazebo's friction alone holds only 73% of pieces
to the end of the carry, and only 10 of 30 cylinders. How well the real
gripper holds is something only hardware can measure.

Step 2's pieces, layout (with the sorting boxes) and reach targets are sized to the SO-101 (reach
targets lie 3–8 cm above the table, step 1's 5–65 cm), so the reach rows
compare each robot on its own workspace.

The step 1 baseline (3-DOF arm, native and emulated images) stays in
[`benchmarks/baseline/`](benchmarks/baseline/); step 1 itself can be checked
out as tag `v0.2.0`.

Compare two runs. Success rates are tested for significance (overlapping 95%
intervals read "not significant"); other rows have no test and read
"not tested". Runs with different robots, grasp modes, seeds, trial counts or
task sets are flagged.

```bash
docker exec ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 run lab_simulation benchmark.py compare /workspace/results/A /workspace/results/B'
```

## Architecture

```
            Gazebo (gz sim)                                    ROS 2
 ┌─────────────────────────────────────┐      ┌────────────────────────────────────────┐
 │ table_world.sdf: table, 3 pieces    │      │ robot_state_publisher                  │
 │                                     │      │   (lab_so101.urdf.xacro) ─► /tf        │
 │ SO-101, spawned from                │      │        │ robot_description            │
 │ robot_description                   │      │        ▼                               │
 │  ├─ gz_ros2_control ◄───────────────┼──────┼─ arm_controller ◄── move_group ◄──┐   │
 │  │   arm + gripper joints           │      │   gripper_controller (MoveIt)     │   │
 │  │   ──► joint_state_broadcaster ───┼──────┼─► /joint_states                   │   │
 │  └─ grasp latch (DetachableJoint) ◄─┼bridge┼── /lab/latch/<piece>/*      benchmark │
 │ piece PosePublishers ───────────────┼bridge┼─► /lab/pieces/<piece>/pose ─► RViz    │
 │ clock ──────────────────────────────┼bridge┼─► /clock                              │
 └─────────────────────────────────────┘      └────────────────────────────────────────┘
```

One description, `src/lab_simulation/urdf/lab_so101.urdf.xacro`, is spawned
into Gazebo and read by RViz and MoveIt, so they cannot drift apart.

### Topics, actions and services

| Name | Type | Purpose |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Arm and gripper joints, from ros2_control |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Arm trajectories (MoveIt executes through it) |
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | Gripper position target (rad) |
| `/move_action` | `moveit_msgs/action/MoveGroup` | Plan and execute with MoveIt |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Frames `world` → `base_link` → … → `grasp_link` |
| `/lab/pieces/<piece>/pose` | `geometry_msgs/msg/PoseStamped` | Live world pose of each piece |
| `/lab/latch/<piece>/{attach,detach}` | `std_msgs/msg/Empty` | Engage or release the grasp latch |
| `/lab/latch/<piece>/state` | `std_msgs/msg/String` | Latch state: `attached` or `detached` |
| `/lab/piece_markers` | `visualization_msgs/msg/MarkerArray` | Table and pieces for RViz |
| `/world/table_world/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | Teleport a piece (benchmark resets) |

The bridge mapping lives in `config/bridge.yaml`.

### The robot

The SO-101 model in `src/so101_description` is generated from the official
URDF ([TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100),
Apache-2.0) by `tools/generate_xacro.py`. Kinematics, masses, inertias,
joint limits and visual meshes are the upstream values, and a test fails if
they drift. The changes are listed in `meshes/SOURCE.md`:

- **Collision boxes** instead of meshes. Mesh collisions slow the simulation
  to about real time; boxes run it several times faster. The jaws get box
  pads, and the moving pad is tilted so that it closes flat on a 3 cm piece.
- **Servo limits** of the STS3215: 2.94 N·m and 4.7 rad/s, with the upstream
  MuJoCo model's joint damping and friction.
- A **`grasp_link`** frame between the open jaws, where task goals put the
  piece.
- The lab's orange and graphite colours.

| Joint | Range (rad) | |
|---|---|---|
| `shoulder_pan` | ±1.92 | Base rotation (positive turns clockwise seen from above) |
| `shoulder_lift` | ±1.75 | |
| `elbow_flex` | ±1.69 | |
| `wrist_flex` | ±1.66 | |
| `wrist_roll` | −2.74 to 2.84 | |
| `gripper` | −0.17 to 1.75 | One moving jaw; 0.6 is open (~45 mm), −0.17 closed |

The arm stands at the centre of the table (top at z = 0.75 m) facing +x. With
the gripper pointing straight down it reaches 0.14–0.22 m from the shoulder
axis at the heights the benchmark uses. The three pieces are sized to its
jaws: each is 3 cm across the side it is gripped on. Three open sorting boxes
(9 cm inside, 2 cm walls), one per piece colour, stand side by side on the
arm's left; pieces are laid out on the rest of its reach. Inside a box the
jaws open only partly to let go, since fully open they would touch the walls.

### Grasping, and the grasp latch

The gripper is a position controller commanded to "closed", so the jaw keeps
squeezing at the servo's torque limit like the real arm. A grasp counts only
if the jaw stalls open (something is between the jaws) and the gripped piece
sits between them; otherwise the trial is a `grasp_miss`.

Gazebo cannot hold a friction grasp for long: with either of its physics
engines, a squeezed piece creeps out of the jaws within 10–40 simulated
seconds. So, in the default **latched** mode, once a grasp has been verified a
Gazebo `DetachableJoint` locks the piece to the gripper until it is placed.
This is the same idea as the `gazebo_grasp_fix` plugin long used with Gazebo
Classic. The **friction** mode adds nothing and is benchmarked for the record
(`GRASP_MODE=friction ./run-benchmark.sh`). Every report states its mode.

## Project structure

```
ai-robotics-lab/
├── .github/workflows/
│   ├── ci.yml                 # Build, test, lint and benchmark smoke test
│   └── docker.yml             # Image build check
├── Dockerfile                 # ROS 2 Jazzy + Gazebo + MoveIt 2 + ros2_control + virtual desktop
├── docker-compose.yml         # Container definition, ports and volumes
├── start-lab.sh               # Build and start everything
├── enter-lab.sh               # Interactive shell in the running container
├── stop-lab.sh                # Stop and remove the container
├── run-benchmark.sh           # Run the benchmark in the container; results in ./results/
├── benchmarks/
│   ├── baseline/              # Step 1 baseline (3-DOF arm, suction)
│   └── step2/                 # Step 2 baseline (SO-101), and the comparison
├── docs/superpowers/          # Design specs and implementation plans
├── LICENSE                    # Apache License 2.0
├── NOTICE                     # Copyright notice
├── scripts/
│   └── start-desktop.sh       # Xvfb + Openbox + x11vnc + noVNC, run in the container
└── src/
    ├── so101_description/     # The SO-101 model
    │   ├── urdf/              # so101_macro.xacro (generated), standalone xacro, upstream URDF
    │   ├── meshes/            # Upstream meshes, their licence and SOURCE.md
    │   └── tools/generate_xacro.py
    ├── so101_moveit_config/   # SRDF, IK, limits, OMPL, controllers, move_group launch
    └── lab_simulation/        # The lab: world, tasks, benchmark
        ├── config/            # RViz, ros_gz_bridge, ros2_control
        ├── lab_simulation/    # Python package
        │   ├── so101_kinematics.py  # Forward and vertical-approach inverse kinematics
        │   ├── layout.py      # Pieces, workspace and seeded sampling
        │   ├── tasks.py       # Reach and pick-and-place trials
        │   ├── metrics.py     # Statistics, reports and comparisons
        │   ├── arm.py         # Moves through MoveIt
        │   ├── gripper.py     # Jaw control and stall detection
        │   ├── scene.py       # Piece poses, grasp latch, resets, MoveIt planning scene
        │   ├── description.py # Builds the robot description
        │   ├── object_markers.py
        │   └── benchmark.py   # Benchmark command line
        ├── launch/            # arm_world.launch.py, benchmark.launch.py
        ├── scripts/           # benchmark.py, move_arm.py, object_markers.py
        ├── test/
        ├── urdf/lab_so101.urdf.xacro  # SO-101 on the table + ros2_control + grasp latch
        └── worlds/table_world.sdf     # Table, pieces, sorting boxes and lighting
```

## Environment notes

**Display.** The container runs its own X server (Xvfb) with Mesa software
OpenGL, a window manager (Openbox), a VNC server (x11vnc) and a browser client
(noVNC). Gazebo and RViz render there, and you view them at
<http://localhost:6080>. A native VNC client can connect to
`vnc://localhost:5900`. The log line `glx: failed to create drisw screen` is
expected under software rendering and harmless.

**Security.** The VNC server has no password. Both ports are therefore
published on `127.0.0.1` only and are not reachable from your network. Do not
change the port bindings to `0.0.0.0` without adding a VNC password.

**Live editing.** `./src` is bind-mounted into the container, and the
workspace is built with `--symlink-install`. Edits to Python, launch, xacro,
SDF and config files apply on the next launch without rebuilding. Run
`colcon build` after adding new files. To change the SO-101 model itself,
edit `tools/generate_xacro.py` and re-run it.

**ROS networking.** `ROS_DOMAIN_ID=42` keeps this lab's ROS 2 traffic separate
from other ROS systems on the same network.

**Reproducibility.** The base image is pinned by digest in the `Dockerfile`.
The comment there explains how to update it.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `start-lab.sh` reports Docker is not running | Start Docker Desktop and retry. |
| noVNC desktop is completely black | Nothing is running yet. Launch `ros2 launch lab_simulation arm_world.launch.py` and RViz appears. |
| <http://localhost:6080> does not load | Run `docker logs ai-robotics-lab`. Restart with `./stop-lab.sh && ./start-lab.sh`. |
| `move_arm.py` stays on "waiting for /joint_states" | The simulation is not running. Start the launch file in another terminal first. |
| `move_arm.py` reports a pose "not reached" | The sim may be running slowly. Check the real-time factor with `gz topic -e -t /world/table_world/stats`. |
| `move_group` prints "No 3D sensor plugin(s) defined for octomap updates" | Expected: the lab has no depth camera yet. |
| `move_group` prints a stack trace when the launch is stopped | A known MoveIt shutdown crash; it happens after everything has stopped and is harmless. |
| `run-benchmark.sh` says a simulation is already running | Stop the interactive launch first (Ctrl-C, or `./stop-lab.sh && ./start-lab.sh`): two simulations would share topics. |

## Status and roadmap

Verified on 7 October 2026 on Apple Silicon (native arm64) with Docker Desktop.

| Capability | Status |
|---|---|
| Reproducible container with browser-based desktop | Working |
| SO-101 arm from its official model, in Gazebo and RViz | Working |
| ros2_control for the arm and gripper inside Gazebo | Working, tracking error ≤ 0.003 rad |
| MoveIt plans and executes every move around the table and pieces | Working |
| Two-finger pick-and-place, visible in RViz | Working, 98% at baseline (latched), 73% friction-only |
| Colour sorting into matching boxes | Working, 93% at baseline |
| Seeded benchmark with committed baselines | Working (see `benchmarks/`) |
| Test suite (unit, consistency, linters) | Working, 571 checks passing |

Planned next steps:

1. **Camera and perception.** An RGB-D camera over the table; find pieces
   from images instead of from the simulator's ground truth.
2. **Instructions.** "Pick up the red cube", typed or spoken, turned into a
   task by a language model.
3. **Learned policies.** Record demonstrations and train a LeRobot policy;
   compare it with the scripted MoveIt pick on this benchmark.

### The pipeline

```
Camera sees table → "pick up the red cube" → AI writes a task plan →
safety check → MoveIt plans a path → arm moves in Gazebo → success or safe stop
```

### Learning path (12 weeks)

| Weeks | Focus |
|---|---|
| 0–2 | ROS 2 basics, shared setup, first simulation |
| 3–4 | Gazebo arm, MoveIt motion basics (done: SO-101 with ros2_control and MoveIt) |
| 5–6 | Vision: the camera finds objects |
| 7–8 | AI: a language model turns instructions into task plans |
| 9–10 | LeRobot demos, safety gate |
| 11–12 | Full instructed-grasp demo in simulation |

## Resources

- [ROS 2 Jazzy tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic/getstarted/)
- [ros_gz bridge](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)
- [MoveIt 2 tutorials](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

## License

Copyright 2026 Straw Hat Labs. Licensed under the
[Apache License, Version 2.0](LICENSE). See [NOTICE](NOTICE).
