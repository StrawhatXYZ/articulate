# Using Articulate

How to set up, run and test the lab. The [README](../README.md) has the short version.

## Requirements

- Docker Desktop with at least 8 GB of memory allocated
  (or Docker Engine on Linux)
- About 12 GB of free disk space: the image, plus 1.5 GB of vision-model
  weights downloaded on first use (kept in the `hf-cache` Docker volume)
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
   `~/articulate`), not under `C:\` or `/mnt/c`. Bind mounts from the
   Windows file system are much slower.
3. Run `./start-lab.sh` and the other commands exactly as on macOS or Linux.
   <http://localhost:6080> opens in your normal Windows browser.

Without WSL, the same steps can be run directly from PowerShell:

```powershell
docker compose up -d --build
docker exec -it articulate bash    # then use the in-container commands below
```

The repository's `.gitattributes` forces LF line endings, so a Windows Git
checkout does not break the scripts that run inside the Linux container.

## Quick start, in detail

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
| `perception` | `false` | Add the overhead RGB-D camera and the perception node (see [Seeing the table](#seeing-the-table)) |

### Seeing the table

![The overhead camera's view with each piece ringed and named by the robot, and its measured position](media/robot-view.png)

```bash
ros2 launch lab_simulation arm_world.launch.py perception:=true
ros2 topic echo /lab/perception/objects     # in a second terminal
```

With `perception:=true` an RGB-D camera looks down on the table from the
front right, and RViz shows its picture in the **Camera** panel. On request,
the perception node (`src/lab_perception`) turns a frame into named
objects, published as `vision_msgs/Detection3DArray`:

1. **Find** (depth geometry): pixels standing more than 5 mm above the
   table, inside the arm's reach and outside the boxes, are grouped into
   objects. Each object's position and angle come from its top surface.
2. **Name** (SigLIP 2): each object's colour crop is compared with text
   prompts such as "a photo of a red cube"; the most similar wins. A weak or
   ambiguous match is reported as `unknown` and never picked. Adding an
   object type needs only a new prompt, not new training data.

The first launch downloads the model (1.5 GB). The node runs with the
`/opt/lab-ai` interpreter, which has CPU PyTorch.

To make the robot act on what it sees, run the benchmark with
`perception:=vision` (see [Benchmark](benchmarks.md)):

```bash
ros2 launch lab_simulation benchmark.launch.py task:=sort trials:=3 perception:=vision rviz:=true realtime:=true
```

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

### Talking to the robot

Type or say an instruction; the robot plans it, checks it and carries it out
in the simulation, or explains why not.

```
"put the red cube back where it was"
  -> Whisper           speech to text                      (host, ~0.5 s on an M4)
  -> Gemma 4 E2B       sentence + what the robot sees -> tool calls (host GPU, ~5-9 s)
  -> safety check      whole plan validated before anything moves (container)
  -> MoveIt + arm      each step, looking at the table again first
```

**One-time setup on the host.** Install [Ollama](https://ollama.com) and
pull the model (4.6 GB): `ollama pull gemma4:e2b`. For speech, create the
voice client's environment:

```bash
# macOS (Apple Silicon)
python3 -m venv host/.venv && host/.venv/bin/pip install -r host/requirements-mac.txt
# Windows (PowerShell): faster-whisper, NVIDIA GPU if present
py -m venv host\.venv; host\.venv\Scripts\pip install -r host\requirements.txt
```

**Run.** Start the robot in the container and wait for
`[instruction]: ready`, open <http://localhost:6080>, then talk to it from a
host terminal:

```bash
./enter-lab.sh
ros2 launch lab_simulation demo.launch.py      # perception:=ground_truth, rviz:=false, speed:=0.5

host/.venv/bin/python host/voice.py            # press Enter, speak, press Enter
host/.venv/bin/python host/voice.py --model turbo --language en   # more accurate hearing
host/.venv/bin/python host/voice.py --text "swap the cube and the cylinder"
```

The transcript, the plan, each step and the result are printed as they
happen. Without the voice client, send a goal from the container:
`ros2 action send_goal --feedback /lab/instruct lab_interfaces/action/Instruct "{text: 'go home'}"`.

**What it can do.** Put a piece in its box; put it back where it started
(picking it out of the box, from where the robot remembers putting it);
put it next to another piece, on a given side; move it to an area of the
table; swap two pieces; sort everything; go home; answer questions about the
table ("which pieces are in their boxes?"). Directions are the robot's:
front is away from it. The boxes fill the robot's left, so the free table is
a thin strip: an area is a side along it (left, center, right) or a depth
across it (front, back), and each third of the strip holds about one piece.

**What it refuses.** Unsafe or destructive requests ("throw the cube on the
floor"), objects that do not exist ("the purple pyramid"), requests
unrelated to the pieces, and anything the safety check cannot place safely
("no free spot"). The model never gives coordinates; the check validates
the whole plan first and the executor stops at the first step that is no
longer valid. Every request ends with one outcome: `done`, `answered`,
`refused`, `not_understood`, `failed`, `planner_unavailable` or
`cancelled`; a request sent while one runs is rejected (busy). Each request
is logged to `results/instructions.jsonl`.

**Limits.** Each request stands alone: name the piece ("put the red cube
back"), not "put it back". A request takes 15-40 s, mostly arm motion; the
demo arm runs at 0.8 of its speed limits (the pick and sort benchmarks at
0.5, so their numbers stay comparable). Whisper's `small` model can mishear
accented speech ("sort" as "shut"): `--model turbo` and `--language` help.
Before a demo on a 16 GB Mac, restart Docker Desktop and close spare browser
tabs: when macOS swaps, planning slows from about 4 s to 6-9 s. What we tried
to make requests faster, and what we kept, is in
[`benchmarks/m3b-speed/`](../benchmarks/m3b-speed/comparison.md).

### Running the tests

```bash
cd /workspace
colcon test --packages-select lab_simulation
colcon test-result --verbose
```

Use `colcon test` without `--packages-select` to test every package.
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
  The M3b tests cover the safety check (including thousands of random,
  malformed model replies: it must refuse, never crash), the planner against
  a fake Ollama server, the placement memory, the executor and the
  instruction scorer.

The host voice client has its own tests: `host/.venv/bin/python -m pytest -q host`.

### Continuous integration

`.github/workflows/ci.yml` runs on every push to `main` and on every pull
request:

- **Build and test** builds every package and runs the full test suite
  in the official `ros:jazzy` image, then the voice client's tests.
- **Lint** runs ShellCheck on the shell scripts and Hadolint on the
  Dockerfile. Rules ignored on purpose are listed, with reasons, in
  `.hadolint.yaml`.
- **Benchmark smoke test** runs 5 trials per task in a headless simulation and
  uploads the report as the `benchmark-report` artifact.

`.github/workflows/docker.yml` builds the image whenever the `Dockerfile`, the
compose file, the scripts or the package change.

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
| <http://localhost:6080> does not load | Run `docker logs articulate`. Restart with `./stop-lab.sh && ./start-lab.sh`. |
| `move_arm.py` stays on "waiting for /joint_states" | The simulation is not running. Start the launch file in another terminal first. |
| `move_arm.py` reports a pose "not reached" | The sim may be running slowly. Check the real-time factor with `gz topic -e -t /world/table_world/stats`. |
| `move_group` prints "No 3D sensor plugin(s) defined for octomap updates" | Expected: the lab has no depth camera yet. |
| `move_group` prints a stack trace when the launch is stopped | A known MoveIt shutdown crash; it happens after everything has stopped and is harmless. |
| `run-benchmark.sh` says a simulation is already running | Stop the interactive launch first (Ctrl-C, or `./stop-lab.sh && ./start-lab.sh`): two simulations would share topics. |

## Resources

- [ROS 2 Jazzy tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic/getstarted/)
- [ros_gz bridge](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge)
- [MoveIt 2 tutorials](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
