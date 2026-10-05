# AI & Robotics Lab

[![CI](https://github.com/StrawhatXYZ/ai-robotics-lab/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/StrawhatXYZ/ai-robotics-lab/actions/workflows/ci.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

A containerised ROS 2 simulation lab for building an **instructed grasp**
system: a robot arm that picks up objects on spoken or written command,
developed in simulation first.

Phase 1 provides a 3-DOF arm with a simulated suction gripper on a table with
graspable objects in Gazebo, bridged to ROS 2 and visualised in RViz, plus a
seeded benchmark that measures reach and pick-and-place performance. Everything runs inside Docker and is
viewed in a web browser, so no host-side ROS, X server or GPU is required.

| Component | Version | Role |
|---|---|---|
| ROS 2 | Jazzy | Middleware connecting perception, planning and control |
| Gazebo | Harmonic (gz-sim 8) | Physics simulation of the arm, table and objects |
| MoveIt 2 | Jazzy | Collision-aware motion planning (installed; integration is the next milestone) |
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
# Terminal 1: start the simulation (Gazebo server, bridge, TF, RViz)
./enter-lab.sh
ros2 launch lab_simulation arm_world.launch.py

# Terminal 2: run the arm demonstration
./enter-lab.sh
ros2 run lab_simulation move_arm.py
```

RViz shows the arm stepping through five poses. The demo verifies each pose
against `/joint_states` and exits with status 0 only if all were reached.

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
| `rviz` | `true` | Start RViz 2 with the preconfigured arm view |
| `gz_gui` | `false` | Also start the Gazebo GUI. It uses several CPU cores under software rendering, so RViz is the lighter viewer. |
| `use_sim_time` | `true` | Drive ROS nodes from the Gazebo `/clock` |

### Commanding joints directly

Each joint accepts a position target in radians. Targets are latched, so a
single message is enough to move and hold a joint:

```bash
ros2 topic pub --once /simple_arm/joint_2/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"
ros2 topic echo /joint_states
```

### Watching a pick

RViz shows the table and the pieces at their live positions. To watch the arm
pick and place pieces at normal speed, run inside the container:

```bash
ros2 launch lab_simulation benchmark.launch.py task:=pick_place trials:=5 rviz:=true realtime:=true
```

and open <http://localhost:6080>. Without `realtime:=true` the benchmark runs
several times faster than real time.

### Running the tests

```bash
cd /workspace
colcon test --packages-select lab_simulation
colcon test-result --verbose
```

The suite runs the ROS 2 linters (flake8, pep257, xmllint, lint_cmake) and
the pytest files in `test/`: unit tests for the kinematics, layout sampling,
metrics, tasks, arm motion and benchmark runner, and
`test/test_robot_description.py`, which checks that:

- the URDF and SDF agree on every link and joint (geometry, inertia, origins,
  axes, limits, dynamics);
- each inertia tensor matches its link's geometry and mass;
- the arm's placement in the world matches the URDF;
- every joint controller topic is bridged and its output is clamped to the
  joint's effort limit;
- the demo sequence stays within the URDF joint limits;
- every piece publishes its pose, and its suction joint and topics are wired
  through the bridge.

### Continuous integration

`.github/workflows/ci.yml` runs on every push to `main` and on every pull
request:

- **Build and test** builds the package and runs the full test suite in the
  official `ros:jazzy` image.
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
./run-benchmark.sh                              # both tasks, 100 trials each, seed 42
TASK=pick_place TRIALS=10 ./run-benchmark.sh    # quick check
```

| Task | What happens | Success means |
|---|---|---|
| `reach` | The tip moves to a random reachable point | Tip within 1 cm |
| `pick_place` | Suction pick of a random piece, placed at a random target | Grasped within 5 mm, carried without separating (3 cm), placed within 2 cm |

Failures are classified as `unreachable`, `grasp_miss`, `dropped`,
`placement_error` or `timeout`. Success rates carry 95% Wilson intervals.
Robot timings are in simulation seconds; trials per minute and real-time
factor are wall-clock. The benchmark lifts Gazebo's real-time cap unless
`realtime:=true`.

### Step 1 baseline

Seed 42, 100 trials per task, commit `dca2d0c`
([`benchmarks/baseline/`](benchmarks/baseline/)):

| | Native arm64 (Apple Silicon) | Emulated amd64 (previous image) |
|---|---|---|
| Reach success | 100% (95% CI 96–100%) | 100% (96–100%) |
| Reach tip error, mean | 2.3 mm | 2.2 mm |
| Pick-and-place success | **42%** (33–52%) | 43% (34–53%) |
| Pick-and-place failures | dropped 37, placement error 21 | dropped 36, placement error 21 |
| Pick-and-place cycle time, mean | 15.6 s | 15.6 s |
| Real-time factor | 4.9× | 3.6× |
| Trials per minute | **26.6** | 19.1 |

The native image runs the benchmark **1.4× faster** with the same results.
Most pick failures happen during the carry: the 3-DOF arm sags under load,
and a held piece sways or tilts on the compliant suction joint. These are the
numbers later steps (SO-101 arm, ros2_control, MoveIt) are measured against.

Compare two runs (changes inside the confidence interval are marked
"not significant"):

```bash
docker exec ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 run lab_simulation benchmark.py compare /workspace/results/A /workspace/results/B'
```

## Architecture

```
                      Gazebo (gz sim)                              ROS 2
 ┌─────────────────────────────────────────┐          ┌──────────────────────────────┐
 │ table_world.sdf                         │          │                              │
 │  └─ simple_arm.sdf                      │          │  move_arm.py ──┐             │
 │      ├─ JointPositionController ×3 ◄────┼─ bridge ─┼── /simple_arm/joint_N/cmd_pos│
 │      └─ JointStatePublisher (50 Hz) ────┼─ bridge ─┼─► /joint_states ──┐          │
 │  clock ─────────────────────────────────┼─ bridge ─┼─► /clock          ▼          │
 └─────────────────────────────────────────┘          │      robot_state_publisher   │
                                                      │      (simple_arm.urdf)       │
                                                      │               │ /tf          │
                                                      │               ▼              │
                                                      │             RViz 2           │
                                                      └──────────────────────────────┘
```

### Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/simple_arm/joint_{1,2,3}/cmd_pos` | `std_msgs/msg/Float64` | ROS → Gazebo | Joint position targets (rad) |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | Joint positions and velocities, 50 Hz |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | Simulation time |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS | Frames `world` → `base_link` → … → `end_effector` |
| `/robot_description` | `std_msgs/msg/String` | ROS | URDF, latched |
| `/lab/pieces/<piece>/pose` | `geometry_msgs/msg/PoseStamped` | Gazebo → ROS | Live world pose of each piece |
| `/simple_arm/suction/<piece>/{attach,detach}` | `std_msgs/msg/Empty` | ROS → Gazebo | Engage or release suction on a piece |
| `/simple_arm/suction/<piece>/state` | `std_msgs/msg/String` | Gazebo → ROS | Suction state: `attached` or `detached` |
| `/lab/piece_markers` | `visualization_msgs/msg/MarkerArray` | ROS | Table and pieces for RViz |
| `/world/table_world/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | service | Teleport a piece (benchmark resets) |

The bridge mapping lives in `config/bridge.yaml`.

### The robot model

The arm is described twice, as is usual for Gazebo-based ROS projects:

- **`worlds/simple_arm.sdf`** is what Gazebo simulates. It carries the joint
  position controllers and the joint state publisher. It is included directly
  in the world file, so its plugins load with the world.
- **`urdf/simple_arm.urdf`** is what ROS tools read: robot_state_publisher,
  RViz and, later, MoveIt.

The test suite fails if the two drift apart. The arm itself:

| Joint | Type | Axis | Range (rad) | Effort (N·m) | Link driven | Length |
|---|---|---|---|---|---|---|
| `joint_1` | revolute | Z (base yaw) | ±π | 20 | `link_1` | 0.25 m |
| `joint_2` | revolute | Y (shoulder pitch) | ±3π/4 (±135°) | 15 | `link_2` | 0.20 m |
| `joint_3` | revolute | Y (elbow pitch) | ±3π/4 (±135°) | 10 | `link_3` | 0.15 m |

Each link frame sits on the joint that drives it, with geometry offset by half
the link length, so links stack end to end. `base_link` is welded to the table
top at z = 0.75 m. The controllers' PID gains are tuned per joint to the load
each one carries. Inertias are computed from each link's solid primitive.
The shoulder sits 0.29 m above the table, so reaching the table needs the
shoulder and elbow to lean past horizontal: hence the ±135° range.

The table holds three flat-topped 100 g pieces (a red cube, a blue cylinder
and a green block) as grasp targets. Suction is simulated with one Gazebo
`DetachableJoint` per piece, declared in the world file. It engages only when
the tip is within 5 mm of a piece's top, and the code waits for the plugin to
confirm `attached` or `detached`.

## Project structure

```
ai-robotics-lab/
├── .github/workflows/
│   ├── ci.yml                 # Build, test, lint and benchmark smoke test
│   └── docker.yml             # Image build check
├── Dockerfile                 # ROS 2 Jazzy + Gazebo + MoveIt 2 + virtual desktop
├── docker-compose.yml         # Container definition, ports and volumes
├── start-lab.sh               # Build and start everything
├── enter-lab.sh               # Interactive shell in the running container
├── stop-lab.sh                # Stop and remove the container
├── run-benchmark.sh           # Run the benchmark in the container; results in ./results/
├── benchmarks/
│   └── baseline/              # Committed step 1 baseline (native, emulated, comparison)
├── LICENSE                    # Apache License 2.0
├── NOTICE                     # Copyright notice
├── scripts/
│   └── start-desktop.sh       # Xvfb + Openbox + x11vnc + noVNC, run in the container
└── src/
    └── lab_simulation/        # ROS 2 package (ament_cmake)
        ├── package.xml
        ├── CMakeLists.txt
        ├── config/
        │   ├── arm.rviz           # RViz display configuration
        │   └── bridge.yaml        # ros_gz_bridge topic map
        ├── lab_simulation/        # Python package
        │   ├── kinematics.py      # Closed-form forward and inverse kinematics
        │   ├── layout.py          # Piece geometry and seeded sampling
        │   ├── tasks.py           # Reach and pick-and-place trials
        │   ├── metrics.py         # Statistics, reports and comparisons
        │   ├── arm.py             # Smooth joint motion and settle detection
        │   ├── scene.py           # Piece poses, suction and scene reset
        │   ├── object_markers.py  # RViz markers for the table and pieces
        │   └── benchmark.py       # Benchmark command line
        ├── launch/
        │   ├── arm_world.launch.py
        │   └── benchmark.launch.py
        ├── scripts/
        │   ├── benchmark.py       # Entry point: run, report, compare
        │   ├── move_arm.py        # Scripted, self-verifying joint demo
        │   └── object_markers.py  # Entry point: RViz piece markers
        ├── test/                  # Unit and consistency tests (pytest)
        ├── urdf/
        │   └── simple_arm.urdf    # Robot description for ROS
        └── worlds/
            ├── simple_arm.sdf     # Robot model for Gazebo
            └── table_world.sdf    # Table, objects, arm and lighting
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
workspace is built with `--symlink-install`. Edits to Python, launch, URDF,
SDF and config files apply on the next launch without rebuilding. Run
`colcon build` after adding new files.

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
| `run-benchmark.sh` says a simulation is already running | Stop the interactive launch first (Ctrl-C, or `./stop-lab.sh && ./start-lab.sh`): two simulations would share topics. |

## Status and roadmap

Verified on 5 October 2026 on Apple Silicon (native arm64) with Docker Desktop.

| Capability | Status |
|---|---|
| Reproducible container with browser-based desktop | Working |
| Table world with arm and graspable objects | Working |
| Arm welded to the table, links stacked end to end | Working |
| Joint position control over ROS 2 topics | Working, smooth eased motion |
| Suction pick-and-place, visible in RViz | Working, 42% success at baseline |
| Seeded benchmark with committed baseline | Working (see `benchmarks/baseline/`) |
| Gazebo GUI | Working natively |
| `/joint_states`, TF and RViz model in the `world` frame | Working |
| Test suite (unit, consistency, linters) | Working, 266 checks passing |
| MoveIt plans and executes trajectories | Next milestone |

Planned next steps:

1. **ros2_control.** Replace the per-joint Gazebo controllers with
   `gz_ros2_control` and a `JointTrajectoryController`, the interface MoveIt
   executes trajectories through.
2. **MoveIt 2.** Generate the MoveIt configuration (SRDF, kinematics, planning
   pipeline) and plan collision-aware motions.
3. **Gripper and camera.** Add a gripper at `end_effector` and an RGB-D camera
   overlooking the table.

### The pipeline

```
Camera sees table → "pick up the red cube" → AI writes a task plan →
safety check → MoveIt plans a path → arm moves in Gazebo → success or safe stop
```

### Learning path (12 weeks)

| Weeks | Focus |
|---|---|
| 0–2 | ROS 2 basics, shared setup, first simulation |
| 3–4 | Gazebo arm, MoveIt motion basics |
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
