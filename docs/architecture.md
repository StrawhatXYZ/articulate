# How Articulate is built

The parts of the lab and how they connect. For using it, see the [guide](guide.md).

## Components

| Component | Version | Role |
|---|---|---|
| ROS 2 | Jazzy | Middleware connecting perception, planning and control |
| Gazebo | Harmonic (gz-sim 8) | Physics simulation of the arm, table and objects |
| ros2_control | Jazzy | Arm and gripper controllers, running inside Gazebo (gz_ros2_control) |
| MoveIt 2 | Jazzy | Collision-aware motion planning for every move |
| RViz 2 | Jazzy | 3D visualisation of the robot model and TF |
| SigLIP 2 | `google/siglip2-base-patch16-224` | Names what the camera sees (zero-shot, CPU, PyTorch) |
| Gemma 4 E2B | `gemma4:e2b` via [Ollama](https://ollama.com), on the host | Plans instructions as tool calls (local, offline) |
| Whisper | mlx-whisper (Apple Silicon) or faster-whisper, on the host | Speech to text for spoken instructions |

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
 │ overhead RGB-D camera ──────────────┼bridge┼─► perception node (SigLIP 2) ──────►  │
 │   (perception:=true)                │      │   /lab/perception/objects ─► benchmark │
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
| `/lab/piece_markers` | `visualization_msgs/msg/MarkerArray` | Table, pieces and boxes for RViz |
| `/lab/camera/{image,depth_image,camera_info}` | `sensor_msgs/msg/Image`, `CameraInfo` | Overhead RGB-D camera (perception only) |
| `/lab/perception/request` | `std_msgs/msg/Empty` | Ask for one detection of the next frame |
| `/lab/perception/objects` | `vision_msgs/msg/Detection3DArray` | Named objects: label, score, 3D pose |
| `/lab/instruct` | `lab_interfaces/action/Instruct` | An instruction in; progress (feedback) and outcome out |
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
articulate/
├── .github/workflows/
│   ├── ci.yml                 # Build, test, lint and benchmark smoke test
│   └── docker.yml             # Image build check
├── Dockerfile                 # ROS 2 Jazzy + Gazebo + MoveIt 2 + ros2_control + virtual desktop
├── docker-compose.yml         # Container definition, ports and volumes
├── start-lab.sh               # Build and start everything
├── enter-lab.sh               # Interactive shell in the running container
├── stop-lab.sh                # Stop and remove the container
├── run-benchmark.sh           # Run the benchmark in the container; results in ./results/
├── AGENTS.md                  # Guide for coding agents and contributors
├── host/                      # Voice client, runs on the host (macOS or Windows)
│   ├── voice.py               # Push to talk -> Whisper -> /lab/instruct
│   ├── stt.py                 # Speech to text: mlx-whisper or faster-whisper
│   └── requirements*.txt
├── benchmarks/
│   ├── baseline/              # Step 1 baseline (3-DOF arm, suction)
│   ├── step2/                 # Step 2 baseline (SO-101), and the comparison
│   ├── m3a/                   # M3a: sorting from the camera vs ground truth
│   └── m3b/                   # M3b: instruction following (planner, end to end)
├── docs/design/              # Design specs and implementation plans, per milestone
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
    ├── lab_interfaces/        # The Instruct action
    ├── lab_perception/        # Camera model, depth segmentation, SigLIP 2 naming, ROS node
    └── lab_simulation/        # The lab: world, tasks, benchmark
        ├── config/            # RViz, ros_gz_bridge, ros2_control
        ├── lab_simulation/    # Python package
        │   ├── so101_kinematics.py  # Forward and vertical-approach inverse kinematics
        │   ├── layout.py      # Pieces, workspace and seeded sampling
        │   ├── tasks.py       # Reach, pick-and-place and sort trials
        │   ├── targets.py     # Instruction target -> safe, reachable table spot
        │   ├── memory.py      # Where the robot believes each piece is
        │   ├── planner.py     # Gemma 4 E2B via Ollama: instruction -> tool calls
        │   ├── plan_check.py  # The safety check: tool calls -> validated steps
        │   ├── executor.py    # Runs checked steps with the pick-and-place skill
        │   ├── instructor.py  # One instruction from words to motion
        │   ├── instruction_cases.py, instruction_eval.py, instruct_task.py  # M3b benchmarks
        │   ├── metrics.py     # Statistics, reports and comparisons
        │   ├── arm.py         # Moves through MoveIt
        │   ├── gripper.py     # Jaw control and stall detection
        │   ├── scene.py       # Piece poses, grasp latch, resets, MoveIt planning scene
        │   ├── description.py # Builds the robot description
        │   ├── object_markers.py
        │   └── benchmark.py   # Benchmark command line
        ├── launch/            # arm_world.launch.py, benchmark.launch.py, demo.launch.py
        ├── scripts/           # benchmark.py, instruction_node.py, instruction_eval.py, ...
        ├── test/
        ├── urdf/lab_so101.urdf.xacro  # SO-101 on the table + ros2_control + grasp latch
        └── worlds/table_world.sdf     # Table, pieces, sorting boxes and lighting
```
