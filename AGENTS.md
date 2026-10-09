# AGENTS.md

Guide for coding agents (Claude Code, Codex, Copilot and others) and
contributors working on this repository. `README.md` says what the lab is;
this file covers how to work on it without breaking it. First written by
@MunuriGangadhar in PR #3.

## What runs where

| Part | Runs in | Python |
|---|---|---|
| Simulation, MoveIt, RViz, ROS 2 nodes (`src/`) | Docker container `ai-robotics-lab` | `/usr/bin/python3` (ROS) |
| Perception node (PyTorch, SigLIP 2) | Same container | `/opt/lab-ai/bin/python` (venv, system site packages) |
| Instruction node (planner, safety check, executor) | Same container | `/usr/bin/python3` (standard library HTTP only) |
| Gemma 4 E2B (the planner's model) | Ollama on the host, reached at `http://host.docker.internal:11434` | – |
| Voice client (`host/`) | The host (macOS or Windows), never the container | `host/.venv` |

`./src` is bind-mounted at `/workspace/src` and built with
`--symlink-install`: Python, launch and config edits apply on the next
launch. New files (scripts, launch files, packages) need `colcon build`.

## Commands

```bash
./start-lab.sh && ./enter-lab.sh                       # host: build and enter the container

# Inside the container
colcon build --symlink-install
colcon test && colcon test-result --verbose            # every check must pass
ros2 launch lab_simulation demo.launch.py              # wait for "[instruction]: ready"
ros2 run lab_simulation instruction_eval.py            # planner-only benchmark (needs Ollama)

# Host
TASK=instruct TRIALS=50 PERCEPTION=vision ./run-benchmark.sh   # end-to-end benchmark
host/.venv/bin/python -m pytest -q host                # voice client tests
host/.venv/bin/python host/voice.py --text "go home"   # talk to the running robot
```

On Windows use `host\.venv\Scripts\python`; from Git Bash, prefix
`docker exec` with `MSYS_NO_PATHCONV=1` so container paths are not rewritten.

## Code conventions

- `ament_flake8` and `ament_pep257` are part of `colcon test`: lines up to
  99 characters, docstrings on public functions, imports in one
  alphabetical group with `lab_simulation` sorted among third-party modules.
- Pure logic stays free of ROS so it can be unit-tested with fakes
  (`test/fakes.py`): `layout`, `targets`, `memory`, `plan_check`,
  `planner`, `executor`, `instructor`, `tasks`, `metrics`. ROS wiring lives
  in `scene.py`, `arm.py`, `gripper.py`, `ros_util.py` and `scripts/`.
- Times in tasks are simulation seconds; use `ros_util.spin_until` and
  `spin_for`, never `time.sleep`.
- Keep the benchmark comparable: `arm.VELOCITY_SCALING` (0.5) is the
  benchmark and demo speed. Grasping changes need at least 20 pick_place
  trials before they are trusted.
- Tests that a change needs: write them first and watch them fail.

## Interfaces to keep stable

- Action `/lab/instruct` (`lab_interfaces/action/Instruct`): goal `text`;
  feedback `stage` (`heard`, `planning`, `plan`, `step`) and `message`;
  result `outcome` (`done`, `answered`, `refused`, `not_understood`,
  `failed`, `planner_unavailable`, `cancelled`), `message`, `steps`. One
  goal at a time; a goal sent while busy is rejected.
- Planner tools (`planner.TOOLS`): `move_piece(piece, target, other_piece,
  side, region)` with target `bin`, `start`, `near` or `region`;
  `swap_pieces`; `sort_all`; `go_home`; `refuse(reason)`; `answer(text)`.
- Table areas are defined once, in `layout` (`REGIONS`, `in_region`,
  `area_of`). Directions are the robot's: front is away from it.
- The 42 held-out cases in `instruction_cases.TEST` are frozen: never edit
  them and never tune the prompt on them.

## Safety rules

- `plan_check.check` decides what may move; never bypass it. It must never
  raise, whatever the model returns (it has a random-input test).
- The model never supplies coordinates. `xy` targets are created only by
  the check (swaps) and resolved through `targets.resolve`.
- In vision mode nothing the planner, check or executor uses may come from
  ground truth; ground truth is for scoring only.
- Refusals are explicit (`refuse`); "no tool called" is `not_understood`,
  not a refusal.

## Gotchas

- **One simulation at a time.** Two launches share topics and corrupt each
  other. Stop everything first (`./stop-lab.sh && ./start-lab.sh` is the
  sure way). `pkill -f` patterns match the shell that runs them: put
  process clean-up in a script file.
- **Gemma 4 thinks by default, and needs to.** Without thinking it writes
  tool calls as plain text and plans much worse. A runaway reply is capped
  (`num_predict` 1024, 30 s timeout, one retry).
- **Ollama must be reachable from the container** at
  `host.docker.internal:11434` (`curl` it from inside). On Linux hosts
  `docker-compose.yml` adds the mapping.
- **The camera does not see inside the boxes** (they are masked). The robot
  remembers what it put there (`memory.py`); the camera wins whenever it
  sees a piece on the table.
- **An idle node has a stale clock.** `ros_util.catch_up` brings it up to
  date before simulation time is measured; keep calling `spin_until`
  rather than reading `sim_now` after a pause.
- **The free table is a thin strip** right of the boxes: each third holds
  about one piece, so "move it to the center" can be refused as full.
- EmbeddingGemma models need a Hugging Face token; SigLIP does not.

## Git

Conventional Commits, one commit per feature, each building and passing its
own tests. Do not commit or push unless asked. Never commit `host/.venv/`,
model weights or `results/`, and never put personal e-mail addresses in
commits or files.
