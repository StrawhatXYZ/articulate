# AGENTS.md

Guide for AI coding agents (Claude Code, Codex, Copilot and others) working on
this repository. Read `README.md` for what the lab is; this file covers how to
work on it without breaking it.

## What runs where

| Part | Runs in | Python |
|---|---|---|
| Simulation, MoveIt, RViz, ROS 2 nodes (`src/`) | Docker container `ai-robotics-lab` | `/usr/bin/python3` (ROS) |
| Perception and command nodes (PyTorch models) | Same container | `/opt/lab-ai/bin/python` (venv with system site-packages) |
| Voice and brain (`host/`) | The host (Windows), never the container | `host\.venv` |
| Gemma 4 E2B (the brain's model) | Ollama on the host, `http://localhost:11434` | – |

`./src` is bind-mounted at `/workspace/src` and built with
`--symlink-install`: Python, launch and config edits apply on the next launch.
New files (scripts, launch files) need `colcon build`.

## Commands

```bash
# Build (inside the container)
source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install

# Tests and linters for the packages you touched
colcon test --packages-select lab_perception lab_simulation && colcon test-result --verbose
cd /workspace/src/lab_simulation && ament_flake8      # must print no errors

# Demo (the brain path)
ros2 launch lab_simulation demo.launch.py              # wait for "[command]: ready"
```

```bat
:: Host side (Windows)
host\.venv\Scripts\python host\brain.py "put the red cube back"       :: plan only, robot not needed
host\.venv\Scripts\python host\voice_command.py --text "go home"      :: send to the running robot
host\.venv\Scripts\python host\voice_command.py --dry-run --text "..."
```

From Git Bash on Windows, prefix `docker exec` with `MSYS_NO_PATHCONV=1`, or
container paths such as `/workspace/...` are rewritten to Windows paths.

## Code conventions

- Style is enforced by `ament_flake8` and `ament_pep257`: lines up to 99
  characters, docstrings on public functions, imports in one alphabetical
  group with `lab_simulation` sorted among third-party modules (see
  `scripts/benchmark.py`).
- Pure logic stays free of ROS so it can be unit-tested: `layout.py`,
  `tasks.py`, `targets.py`, `instruction.py`, `so101_kinematics.py`,
  `metrics.py`. ROS wiring lives in `scene.py`, `arm.py`, `gripper.py` and the
  `scripts/` nodes. Tests use fakes (`test/test_tasks.py`).
- Times in tasks are simulation seconds; use `ros_util.spin_until` and
  `spin_for`, not `time.sleep`.
- Keep the benchmark comparable: `arm.VELOCITY_SCALING` (0.5) is the
  benchmark speed; the demo passes its own `speed`.

## Interfaces you must keep stable

`host/brain.py` and `scripts/command_node.py` talk through these topics
(all `std_msgs/msg/String`):

- `/lab/plan`: `{"text": str, "steps": [STEP, ...]}`, STEP is
  `{"action": "move", "piece": NAME, "to": TARGET}`, `{"action": "home"}` or
  `{"action": "sort_all"}`; TARGET is `{"kind": "bin"}`, `{"kind": "start"}`,
  `{"kind": "near", "piece": OTHER, "side": "left|right|front|back"}`,
  `{"kind": "region", "region": "left|right|front|back|center"}` or
  `{"kind": "xy", "xy": [x, y]}`.
- `/lab/scene`: `{"busy": bool, "pieces": {NAME: {"xy", "start_xy", "in_bin",
  "colour", "shape"}}, "bins": {COLOUR: [x, y]}}`.
- `/lab/instruction`: a plain sentence (the old path, kept on purpose).
- `/lab/instruction/status`: progress lines; `host/voice_command.py` stops
  watching on `plan done`, `done:`, `failed`, `refused`, `did not understand`
  or `busy`.

Directions are the robot's: it faces +x, front is away from it, back is
towards it, left is +y. Pieces: `red_cube`, `blue_cylinder`, `green_block`.
Change these in both places at once, or not at all.

## Safety rules for plans

- The brain refuses unsafe or unrelated requests by calling no tool; a refusal
  must never fall back to `/lab/instruction`. Only an unreachable Ollama
  falls back.
- `command_node.py` validates the whole plan before moving anything.
- `targets.py` only returns spots inside the reachable ring around the pan
  axis, clear of the boxes and other pieces. Do not bypass it.

## Gotchas

- **One simulation at a time.** Two launches share topics. Stop everything
  first: `pkill -INT -f "[r]os2 launch"; sleep 5; pkill -9 -f "[g]z sim"; pkill -9 -f "[m]ove_group"`
  (the brackets stop `pkill -f` from matching its own command line).
- **Do not run `stop-lab.sh` or `docker compose down`** on a container whose
  `/opt/lab-ai` was installed by hand: it is deleted with the container. Use
  `docker stop` / `docker start`.
- **EmbeddingGemma 2 must load in float32** on the CPU: its bfloat16 weights
  are emulated and about four times slower.
- **Vision does not see pieces inside boxes** (the boxes are masked); the
  command node uses the simulator's poses for those.
- **`ros2 topic echo` can wrap a long message**; read up to the `---`
  separator.
- MoveIt's "Invalid goal state" when picking from a box means the gripper
  would touch a wall; `tasks._approach_yaws` tries the other orientations.

## Git

Do not commit or push unless the user asks; the maintainer pushes manually.
Never commit `host/.venv/`, model weights or `results/`.
