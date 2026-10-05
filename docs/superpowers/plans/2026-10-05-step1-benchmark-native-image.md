# Step 1: Native Image, Pick-and-Place and Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the lab natively on arm64, let the 3-DOF arm pick and place pieces with a simulated suction gripper (visible in RViz), and add a seeded benchmark that produces a committed baseline for all later "10x" claims.

**Architecture:** A pure-Python core (`kinematics`, `layout`, `metrics`, `tasks`) holds all logic and is unit-tested without a simulation. Thin ROS adapters (`arm`, `scene`, `ros_util`, `object_markers`) connect it to Gazebo through `ros_gz_bridge`. A `benchmark` CLI drives one long-running headless simulation, resetting the scene between trials, and writes `trials.jsonl`, `summary.json` and `report.md`.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8), ros_gz_bridge, rclpy, Python 3.12 (stdlib only), pytest, Docker (`ros:jazzy` multi-arch), GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-10-05-step1-benchmark-native-image-design.md`

## Global Constraints

- Base image: `ros:jazzy@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca` (OCI index: linux/amd64 + linux/arm64).
- Joint limits: `joint_1` ±3.14159; `joint_2` and `joint_3` ±2.356 rad, identical in URDF, SDF and `move_arm.py`.
- Thresholds: reach success ≤ 0.01 m tip error; grasp gap ≤ 0.005 m; carry deviation ≤ 0.01 m; placement ≤ 0.02 m; disturbed > 0.01 m; piece at rest < 0.001 m/s; reset settle timeout 3 s; step timeout 5 s (simulation time); unresponsive after 5 s without `/joint_states` (wall time).
- Robot timings are simulation seconds; throughput (trials per minute, real-time factor) is wall-clock.
- Benchmarks lift the real-time cap with `set_physics real_time_factor: 1000`; interactive launches keep the cap.
- Seeding: each task uses `random.Random(f'{seed}-{task_name}')`. Baseline: seed 42, 100 trials per task.
- Python: stdlib only in `lab_simulation`; ament flake8 style (single quotes, max line 99, Google import order) and pep257 must pass.
- Commits: Conventional Commits; identity `Sandeep Donepudi <sandeep@strawhat.xyz>` (already set repo-locally); **no Claude/co-author trailers**; never commit `CLAUDE.md`.
- Every commit must build and pass its own tests.

## Review Focus

- A piece knocked off the table or toppled during a trial must produce `placement_error` with the final pose recorded, never a crash (test: Task 6, `test_pick_place_piece_knocked_off_table_is_placement_error`).
- A simulation that stalls mid-run must stop the run, keep every completed trial on disk and exit with code 3 (test: Task 10, `test_unresponsive_simulation_stops_and_keeps_partial_results`).
- Two runs started in the same second must never share or overwrite a results directory (test: Task 10, `test_make_run_dir_never_reuses_a_directory`).
- Comparing runs with different task sets must compare only the common tasks (test: Task 5, `test_compare_only_uses_common_tasks`).
- A run with zero successful trials must still summarise and render, showing `n/a` (test: Task 5, `test_summary_with_no_successes_renders`).

## Conventions Used Below

All commands run on the macOS host from the repository root unless stated. Two helpers:

```bash
# Run pytest directly against the package sources (no build needed).
lab_pytest() { docker exec ai-robotics-lab bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/src/lab_simulation && python3 -m pytest -q $*"; }

# Full build plus the complete colcon test suite (linters and all pytest files).
lab_test() { docker exec ai-robotics-lab bash -c 'source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install && colcon test --packages-select lab_simulation && colcon test-result --verbose'; }
```

Process-management commands must be run from script files or with `docker exec -e`, never as `bash -c` strings containing `pkill -f`/`pgrep -f` patterns (they match their own command line).

## File Map

| File | Status | Responsibility |
|---|---|---|
| `Dockerfile` | modify | Multi-arch `ros:jazzy` base plus `ros-jazzy-desktop` |
| `docker-compose.yml` | modify | Drop amd64 platform; mount `./results` |
| `.gitignore` | modify | Ignore `results/` |
| `run-benchmark.sh` | create | Host wrapper: provenance, then benchmark launch in the container |
| `src/lab_simulation/CMakeLists.txt` | modify | Install Python package and new scripts; run the whole `test/` directory |
| `src/lab_simulation/package.xml` | modify | New build and runtime dependencies |
| `src/lab_simulation/lab_simulation/__init__.py` | create | Package marker |
| `src/lab_simulation/lab_simulation/kinematics.py` | create | Closed-form FK/IK (pure) |
| `src/lab_simulation/lab_simulation/layout.py` | create | Piece geometry, grasp gap, seeded sampling (pure) |
| `src/lab_simulation/lab_simulation/metrics.py` | create | Statistics, summaries, reports, comparisons (pure) |
| `src/lab_simulation/lab_simulation/tasks.py` | create | `ReachTask`, `PickPlaceTask`, `TrialResult`, `MoveResult` (pure) |
| `src/lab_simulation/lab_simulation/ros_util.py` | create | Spinning against simulation time |
| `src/lab_simulation/lab_simulation/arm.py` | create | Joint commands and settle detection |
| `src/lab_simulation/lab_simulation/scene.py` | create | Piece poses, reset, suction |
| `src/lab_simulation/lab_simulation/object_markers.py` | create | RViz markers for table and pieces |
| `src/lab_simulation/lab_simulation/benchmark.py` | create | CLI: run, report, compare |
| `src/lab_simulation/scripts/benchmark.py` | create | Executable entry point |
| `src/lab_simulation/scripts/object_markers.py` | create | Executable entry point |
| `src/lab_simulation/scripts/move_arm.py` | modify | Widened `JOINT_LIMITS` |
| `src/lab_simulation/urdf/simple_arm.urdf` | modify | Widened limits |
| `src/lab_simulation/worlds/simple_arm.sdf` | modify | Widened limits |
| `src/lab_simulation/worlds/table_world.sdf` | modify | Pieces inside reach; a pose publisher per piece; suction plugins in the arm `<include>` |
| `src/lab_simulation/config/bridge.yaml` | modify | Per-piece pose topics, suction topics, `set_pose` service |
| `src/lab_simulation/config/arm.rviz` | modify | MarkerArray display |
| `src/lab_simulation/launch/arm_world.launch.py` | modify | Start `object_markers`; release suction at start-up if needed |
| `src/lab_simulation/launch/benchmark.launch.py` | create | Headless simulation plus benchmark, shut down when done |
| `src/lab_simulation/test/conftest.py` | create | Import package from source |
| `src/lab_simulation/test/test_kinematics.py` | create | |
| `src/lab_simulation/test/test_layout.py` | create | |
| `src/lab_simulation/test/test_metrics.py` | create | |
| `src/lab_simulation/test/test_tasks.py` | create | |
| `src/lab_simulation/test/test_benchmark.py` | create | |
| `src/lab_simulation/test/test_robot_description.py` | modify | Suction, bridge and piece checks; tolerate service entries |
| `.github/workflows/ci.yml` | modify | Headless benchmark smoke job |
| `.github/workflows/docker.yml` | create | Image build check on relevant changes |
| `benchmarks/baseline/` | create | Committed baseline summaries, reports and comparison |
| `README.md` | modify | Benchmark, pick demo, reading reports, GUI status |

---

### Task 1: Probe suction, pose names and services (throwaway)

Answers four questions the design depends on. Nothing from this task is committed.

**Files:** none in the repository. Scratch files live in `/tmp/probe` inside a disposable container.

**Interfaces:**
- Consumes: the current image `ai-robotics-lab:latest` (Gazebo 8.15, same as the native image will have).
- Produces: four recorded answers used by Tasks 7 and 9:
  - `ATTACHED_AT_START` (yes/no): does a `DetachableJoint` attach as soon as the world loads?
  - `ATTACH_WORKS` (yes/no): after a detach, does publishing to `attach_topic` re-attach?
  - `POSE_NAMES_OK` (yes/no): does the bridged `TFMessage` carry `child_frame_id: red_cube`?
  - `SET_POSE_OK` (yes/no): does the bridged `set_pose` service move a piece?

- [ ] **Step 1: Write the probe script**

Create `/tmp/probe.sh` on the host:

```bash
#!/bin/bash
# Throwaway probe for Task 1. Runs inside a disposable container.
set -uo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /tmp/probe/worlds
cp /workspace/src/lab_simulation/worlds/simple_arm.sdf /tmp/probe/worlds/
python3 - <<'EOF'
src = open('/workspace/src/lab_simulation/worlds/table_world.sdf').read()
src = src.replace('<pose>0.25 0.15 0.775 0 0 0</pose>', '<pose>0.173 0.10 0.775 0 0 0</pose>')
plugin = '''
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>end_effector</parent_link>
        <child_model>red_cube</child_model>
        <child_link>cube_link</child_link>
        <attach_topic>/simple_arm/suction/red_cube/attach</attach_topic>
        <detach_topic>/simple_arm/suction/red_cube/detach</detach_topic>
        <suppress_child_warning>true</suppress_child_warning>
      </plugin>
    </include>'''
src = src.replace('    </include>', plugin, 1)
open('/tmp/probe/worlds/table_world.sdf', 'w').write(src)
EOF
cat > /tmp/probe/bridge.yaml <<'EOF'
- ros_topic_name: /world/table_world/dynamic_pose
  gz_topic_name: /world/table_world/dynamic_pose/info
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
- service_name: /world/table_world/set_pose
  ros_type_name: ros_gz_interfaces/srv/SetEntityPose
  gz_req_type_name: gz.msgs.Pose
  gz_rep_type_name: gz.msgs.Boolean
EOF
GZ_SIM_RESOURCE_PATH=/tmp/probe/worlds gz sim -r -s /tmp/probe/worlds/table_world.sdf > /tmp/probe/gz.log 2>&1 &
sleep 15
cube() { gz topic -e -n 1 -t /world/table_world/dynamic_pose/info | grep -A7 'name: "red_cube"' | grep -E ' (x|y|z):' | tr -s ' ' | tr '\n' ' '; echo; }
joint2() { gz topic -t /simple_arm/joint_2/cmd_pos -m gz.msgs.Double -p "data: $1"; sleep 4; }
echo "cube at start:            $(cube)"
joint2 0.6
echo "cube after arm moved:     $(cube)   <- changed => ATTACHED_AT_START=yes"
gz topic -t /simple_arm/suction/red_cube/detach -m gz.msgs.Empty -p ''
echo "detach publish exit code: $?"
sleep 2; joint2 0.0
echo "cube after detach + move: $(cube)   <- unchanged => detach works"
gz topic -t /simple_arm/suction/red_cube/attach -m gz.msgs.Empty -p ''
sleep 2; joint2 0.6
echo "cube after attach + move: $(cube)   <- changed => ATTACH_WORKS=yes"
gz topic -t /simple_arm/suction/red_cube/detach -m gz.msgs.Empty -p ''
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/tmp/probe/bridge.yaml > /tmp/probe/bridge.log 2>&1 &
sleep 5
echo "bridged frame names:"
timeout 10 ros2 topic echo --once /world/table_world/dynamic_pose --field transforms \
  | grep -oE "child_frame_id='[^']+'" | sort -u
ros2 service call /world/table_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: green_sphere, type: 2}, pose: {position: {x: 0.0, y: -0.25, z: 0.79}, orientation: {w: 1.0}}}" \
  | tail -2
sleep 2
gz topic -e -n 1 -t /world/table_world/dynamic_pose/info | grep -A7 'name: "green_sphere"' | grep -E ' y:'
echo "   <- y close to -0.25 => SET_POSE_OK=yes"
```

- [ ] **Step 2: Run the probe in a disposable container**

```bash
docker run --rm --platform linux/amd64 -v "$PWD/src:/workspace/src:ro" -v /tmp/probe.sh:/probe.sh:ro \
  --entrypoint bash ai-robotics-lab:latest /probe.sh
```

Expected: printed cube positions and service output. Record the four answers. `POSE_NAMES_OK=yes` requires `child_frame_id='red_cube'` (or `...::red_cube`) in the list.

- [ ] **Step 3: Decide**

| Result | Action |
|---|---|
| All four yes (any `ATTACHED_AT_START`) | Continue. Task 9 includes the start-up release only if `ATTACHED_AT_START=yes`. |
| `ATTACH_WORKS=no` | **Stop.** Suction needs the spec's kinematic-carry fallback, which changes `scene.py`'s design. Report to the user before continuing. |
| `POSE_NAMES_OK=no` or `SET_POSE_OK=no` | **Stop.** Report the actual output; the bridge mapping in Task 7 needs revisiting with the user. |
| The `-p ''` publish exits non-zero | Retry the probe with `-p ' '`; use whichever works in Task 9's release command. |

---

### Task 2: Native arm64 image

**Files:**
- Modify: `Dockerfile`
- Modify: `docker-compose.yml`

**Interfaces:**
- Consumes: nothing.
- Produces: image `ai-robotics-lab:latest` running natively (`uname -m` = `aarch64` on Apple Silicon); preserved image `ai-robotics-lab:amd64-emulated` for Task 12.

- [ ] **Step 1: Preserve the emulated image**

```bash
docker tag ai-robotics-lab:latest ai-robotics-lab:amd64-emulated
docker image inspect ai-robotics-lab:amd64-emulated --format '{{.Architecture}}'
```

Expected: `amd64`.

- [ ] **Step 2: Switch the base image**

In `Dockerfile`, replace the header comment and `FROM` line (everything above `# ── MoveIt 2, ros2_control, ...`) with:

```dockerfile
# AI & Robotics Lab: ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2, with an
# in-container virtual desktop (Xvfb + Openbox + x11vnc + noVNC).
#
# The base image is pinned by its multi-architecture index digest, so builds
# are reproducible and run natively on both arm64 (Apple Silicon) and amd64.
# To update: docker buildx imagetools inspect ros:jazzy, then copy the
# "Digest" of the index (MediaType ...image.index...).
FROM ros:jazzy@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca
```

In the `apt-get install` list, add `ros-jazzy-desktop \` as the first package (above `ros-jazzy-moveit \`).

- [ ] **Step 3: Remove the platform pin**

In `docker-compose.yml`, delete these three lines:

```yaml
    # The osrf/ros desktop base image is published for amd64 only. Docker
    # Desktop emulates it on Apple Silicon.
    platform: linux/amd64
```

- [ ] **Step 4: Lint the Dockerfile at CI's threshold**

```bash
docker run --rm -i -v "$PWD/.hadolint.yaml:/.config/hadolint.yaml:ro" hadolint/hadolint hadolint --failure-threshold info - < Dockerfile
```

Expected: no output, exit 0.

- [ ] **Step 5: Rebuild and start natively**

```bash
./stop-lab.sh
./start-lab.sh
docker exec ai-robotics-lab uname -m
```

Expected: `start-lab.sh` completes with no "platform does not match" warning; `uname -m` prints `aarch64`. The build takes several minutes (full apt install).

- [ ] **Step 6: Verify nothing regressed**

```bash
lab_test
```

Expected: `Summary: 32 tests, 0 errors, 0 failures, 0 skipped`.

Then, in two terminals: `./enter-lab.sh` → `ros2 launch lab_simulation arm_world.launch.py`; second `./enter-lab.sh` → `ros2 run lab_simulation move_arm.py`. Expected: RViz shows the arm at `http://localhost:6080`; `move_arm.py` ends with `demonstration complete, all poses reached` and exit status 0. Stop the launch with Ctrl-C.

- [ ] **Step 7: Commit**

```bash
git add Dockerfile docker-compose.yml
git commit -m "build: run natively on arm64 with the multi-arch ros:jazzy image"
```

---

### Task 3: Python package and kinematics (with widened joint limits)

**Files:**
- Create: `src/lab_simulation/lab_simulation/__init__.py`, `src/lab_simulation/lab_simulation/kinematics.py`
- Create: `src/lab_simulation/test/conftest.py`, `src/lab_simulation/test/test_kinematics.py`
- Modify: `src/lab_simulation/urdf/simple_arm.urdf`, `src/lab_simulation/worlds/simple_arm.sdf`, `src/lab_simulation/scripts/move_arm.py`
- Modify: `src/lab_simulation/CMakeLists.txt`, `src/lab_simulation/package.xml`

**Interfaces:**
- Consumes: nothing.
- Produces (`lab_simulation.kinematics`): constants `BASE_Z`, `SHOULDER_HEIGHT`, `UPPER_ARM`, `FOREARM`, `TIP_RADIUS`, `JOINT_LIMITS`; `class Unreachable(ValueError)`; `class Joints(NamedTuple: q1, q2, q3)`; `forward(q) -> (x, y, z)`; `within_limits(q) -> bool`; `inverse(xyz) -> Joints` (raises `Unreachable`); `is_reachable(xyz, margin=0.0) -> bool`.

- [ ] **Step 1: Write the test scaffolding and failing tests**

`src/lab_simulation/test/conftest.py`:

```python
"""Make the in-source lab_simulation package importable from the tests."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

`src/lab_simulation/test/test_kinematics.py`:

```python
"""Tests for the closed-form arm kinematics."""

import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from lab_simulation import kinematics
from lab_simulation.kinematics import forward, inverse, is_reachable, Unreachable
import pytest

URDF = ET.parse(Path(__file__).resolve().parents[1] / 'urdf' / 'simple_arm.urdf').getroot()


def joint(name):
    return next(j for j in URDF.findall('joint') if j.get('name') == name)


def origin_z(name):
    return float(joint(name).find('origin').get('xyz').split()[2])


def test_constants_match_urdf():
    assert kinematics.BASE_Z == pytest.approx(origin_z('world_joint'))
    assert kinematics.SHOULDER_HEIGHT == pytest.approx(
        origin_z('joint_1') + origin_z('joint_2'))
    assert kinematics.UPPER_ARM == pytest.approx(origin_z('joint_3'))
    assert kinematics.FOREARM == pytest.approx(origin_z('end_effector_joint'))
    tip = next(link for link in URDF.findall('link') if link.get('name') == 'end_effector')
    assert kinematics.TIP_RADIUS == pytest.approx(
        float(tip.find('visual/geometry/sphere').get('radius')))
    for name, (lo, hi) in zip(('joint_1', 'joint_2', 'joint_3'), kinematics.JOINT_LIMITS):
        limit = joint(name).find('limit')
        assert (lo, hi) == pytest.approx((float(limit.get('lower')), float(limit.get('upper'))))


def test_forward_home_points_straight_up():
    assert forward((0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, 0.75 + 0.29 + 0.35))


def test_round_trip_over_random_configurations():
    rng = random.Random(0)
    for _ in range(2000):
        q = tuple(rng.uniform(lo, hi) for lo, hi in kinematics.JOINT_LIMITS)
        p = forward(q)
        if math.hypot(p[0], p[1]) < 1e-3:
            continue
        solution = inverse(p)
        assert kinematics.within_limits(solution)
        assert forward(solution) == pytest.approx(p, abs=1e-9)


def test_inverse_prefers_the_higher_elbow():
    q = inverse((0.2, 0.0, 0.825))
    elbow_z = 0.75 + 0.29 + 0.20 * math.cos(q.q2)
    assert elbow_z > 0.9


@pytest.mark.parametrize('xyz', [(1.0, 0.0, 1.0), (0.0, 0.0, 0.5), (0.0, 0.0, 2.0)])
def test_out_of_workspace_raises(xyz):
    with pytest.raises(Unreachable):
        inverse(xyz)


def test_is_reachable_applies_margin():
    near_edge = forward((0.0, 1.4, 0.3))  # about 4 mm inside the outer boundary
    assert is_reachable(near_edge)
    assert not is_reachable(near_edge, margin=0.01)


def test_point_directly_behind_the_base_is_reachable():
    q = inverse((-0.2, 0.0, 0.9))
    assert kinematics.within_limits(q)
    assert forward(q) == pytest.approx((-0.2, 0.0, 0.9), abs=1e-5)
```

- [ ] **Step 2: Run to verify failure**

```bash
lab_pytest test/test_kinematics.py
```

Expected: collection error `ModuleNotFoundError: No module named 'lab_simulation'`.

- [ ] **Step 3: Widen the shoulder and elbow limits**

`src/lab_simulation/urdf/simple_arm.urdf`: in the `joint_2` and `joint_3` `<limit>` elements, replace `lower="-1.5708" upper="1.5708"` with `lower="-2.356" upper="2.356"` (exactly two occurrences).

`src/lab_simulation/worlds/simple_arm.sdf`: replace every `<lower>-1.5708</lower>` with `<lower>-2.356</lower>` and every `<upper>1.5708</upper>` with `<upper>2.356</upper>` (two each, `joint_2` and `joint_3`).

`src/lab_simulation/scripts/move_arm.py`: in `JOINT_LIMITS`, change the `joint_2` and `joint_3` entries to `(-2.356, 2.356)`.

In both robot description header comments, after the joint list, add one line: `Shoulder and elbow range is ±135°: with ±90° the table is unreachable.`

- [ ] **Step 4: Write the package**

`src/lab_simulation/lab_simulation/__init__.py`:

```python
"""Simulation, kinematics and benchmark tools for the AI & Robotics Lab."""
```

`src/lab_simulation/lab_simulation/kinematics.py`:

```python
"""
Closed-form kinematics for the 3-DOF lab arm.

Pure Python (no ROS). Positions are in the world frame in metres, angles in
radians. The constants mirror urdf/simple_arm.urdf; test_kinematics.py checks
them against the file.
"""

import math
from typing import NamedTuple, Sequence, Tuple

BASE_Z = 0.75            # world_joint: base_link on the table top
SHOULDER_HEIGHT = 0.29   # joint_1 (0.04) + link_1 (0.25), above base_link
UPPER_ARM = 0.20         # joint_2 -> joint_3
FOREARM = 0.15           # joint_3 -> end_effector
TIP_RADIUS = 0.025       # end_effector sphere

JOINT_LIMITS = (
    (-3.14159, 3.14159),
    (-2.356, 2.356),
    (-2.356, 2.356),
)


class Unreachable(ValueError):
    """Raised when no joint configuration within limits reaches a point."""


class Joints(NamedTuple):
    """Joint angles in radians: base yaw, shoulder pitch, elbow pitch."""

    q1: float
    q2: float
    q3: float


def forward(q: Sequence[float]) -> Tuple[float, float, float]:
    """Return the end-effector centre for joint angles q."""
    q1, q2, q3 = q
    r = UPPER_ARM * math.sin(q2) + FOREARM * math.sin(q2 + q3)
    h = UPPER_ARM * math.cos(q2) + FOREARM * math.cos(q2 + q3)
    return (r * math.cos(q1), r * math.sin(q1), BASE_Z + SHOULDER_HEIGHT + h)


def within_limits(q: Sequence[float]) -> bool:
    """Return True if every joint angle is inside its limit."""
    return all(lo <= v <= hi for v, (lo, hi) in zip(q, JOINT_LIMITS))


def inverse(xyz: Sequence[float]) -> Joints:
    """
    Return joint angles that put the end-effector centre at xyz.

    Of the two elbow solutions, the one with the higher elbow is preferred,
    which keeps the forearm away from the table.
    """
    x, y, z = xyz
    r = math.hypot(x, y)
    h = z - BASE_Z - SHOULDER_HEIGHT
    # atan2 can return exactly +/-pi, a hair outside the +/-3.14159 limit.
    lo, hi = JOINT_LIMITS[0]
    q1 = min(max(math.atan2(y, x), lo), hi) if r > 1e-9 else 0.0
    c3 = (r * r + h * h - UPPER_ARM ** 2 - FOREARM ** 2) / (2 * UPPER_ARM * FOREARM)
    if not -1.0 <= c3 <= 1.0:
        raise Unreachable(f'{tuple(xyz)} is outside the arm workspace')
    solutions = []
    for q3 in (math.acos(c3), -math.acos(c3)):
        q2 = math.atan2(r, h) - math.atan2(
            FOREARM * math.sin(q3), UPPER_ARM + FOREARM * math.cos(q3))
        q = Joints(q1, q2, q3)
        if within_limits(q):
            solutions.append(q)
    if not solutions:
        raise Unreachable(f'{tuple(xyz)} needs joint angles outside the limits')
    return max(solutions, key=lambda q: math.cos(q.q2))


def is_reachable(xyz: Sequence[float], margin: float = 0.0) -> bool:
    """Return True if xyz and points margin away radially and vertically are reachable."""
    x, y, z = xyz
    r = math.hypot(x, y)
    ux, uy = (x / r, y / r) if r > 1e-9 else (1.0, 0.0)
    probes = (
        (0.0, 0.0, 0.0),
        (margin * ux, margin * uy, 0.0),
        (-margin * ux, -margin * uy, 0.0),
        (0.0, 0.0, margin),
        (0.0, 0.0, -margin),
    )
    for dx, dy, dz in probes:
        try:
            inverse((x + dx, y + dy, z + dz))
        except Unreachable:
            return False
    return True
```

- [ ] **Step 5: Run the kinematics tests**

```bash
lab_pytest test/test_kinematics.py
```

Expected: 9 passed.

- [ ] **Step 6: Install the package and run the whole test directory**

In `src/lab_simulation/CMakeLists.txt`:
- After `find_package(ament_cmake REQUIRED)`, add `find_package(ament_cmake_python REQUIRED)`.
- After the `install(DIRECTORY ...)` block, add:

```cmake
# The lab_simulation Python package (kinematics, tasks, benchmark, ...).
ament_python_install_package(${PROJECT_NAME})
```

- Replace `ament_add_pytest_test(test_robot_description test/test_robot_description.py)` with:

```cmake
  ament_add_pytest_test(lab_simulation_tests test TIMEOUT 300)
```

In `src/lab_simulation/package.xml`, after `<buildtool_depend>ament_cmake</buildtool_depend>` add `<buildtool_depend>ament_cmake_python</buildtool_depend>`.

- [ ] **Step 7: Full suite**

```bash
lab_test
```

Expected: 0 failures (linters plus all pytest files, including the existing robot-description tests with the widened limits).

- [ ] **Step 8: Commit**

```bash
git add src/lab_simulation
git commit -m "feat(kinematics): add closed-form arm kinematics and widen shoulder/elbow to ±135°

With ±90° limits no table position is reachable. The Python package is
installed with ament_cmake_python and the whole test directory now runs."
```

---

### Task 4: Layout sampling

**Files:**
- Create: `src/lab_simulation/lab_simulation/layout.py`
- Create: `src/lab_simulation/test/test_layout.py`

**Interfaces:**
- Consumes: `kinematics.is_reachable`, `kinematics.TIP_RADIUS`.
- Produces (`lab_simulation.layout`): constants `TABLE_TOP`, `HOVER`, `LIFT`, `MARGIN`, `MIN_SEPARATION`, `MIN_RADIUS`, `MAX_RADIUS`, `MIN_TRAVEL`, `GRASP_GAP_MAX`; `class Piece(NamedTuple: name, half_height, footprint)` with properties `centre_z`, `grasp_z`; `PIECES: Dict[str, Piece]` keyed `red_cube`, `blue_cylinder`, `green_sphere`; `grasp_gap(piece, piece_xyz, tip_xyz) -> float`; `graspable(piece, xy) -> bool`; `sample_layout(rng, min_separation=MIN_SEPARATION, max_attempts=10000) -> Dict[str, (x, y)]`; `sample_place_target(rng, name, positions, max_attempts=10000) -> (x, y)`; `sample_reach_target(rng, max_attempts=10000) -> (x, y, z)`. Sampling functions raise `RuntimeError` when no valid sample is found.

- [ ] **Step 1: Write the failing tests**

`src/lab_simulation/test/test_layout.py`:

```python
"""Tests for piece geometry and seeded sampling."""

import math
import random

from lab_simulation import layout
from lab_simulation.kinematics import is_reachable
import pytest


def test_same_seed_gives_same_layout():
    assert layout.sample_layout(random.Random('42')) == layout.sample_layout(random.Random('42'))


@pytest.mark.parametrize('seed', range(50))
def test_layout_constraints(seed):
    positions = layout.sample_layout(random.Random(seed))
    assert set(positions) == set(layout.PIECES)
    xys = list(positions.values())
    for i, a in enumerate(xys):
        assert math.hypot(*a) >= layout.MIN_RADIUS
        for b in xys[i + 1:]:
            assert math.dist(a, b) >= layout.MIN_SEPARATION
    for name, (x, y) in positions.items():
        piece = layout.PIECES[name]
        assert is_reachable((x, y, piece.grasp_z), layout.MARGIN)
        assert is_reachable((x, y, piece.grasp_z + layout.HOVER), layout.MARGIN)


@pytest.mark.parametrize('seed', range(50))
def test_place_target_constraints(seed):
    rng = random.Random(seed)
    positions = layout.sample_layout(rng)
    target = layout.sample_place_target(rng, 'red_cube', positions)
    assert math.dist(target, positions['red_cube']) >= layout.MIN_TRAVEL
    for name in ('blue_cylinder', 'green_sphere'):
        assert math.dist(target, positions[name]) >= layout.MIN_SEPARATION
    assert is_reachable((*target, layout.PIECES['red_cube'].grasp_z), layout.MARGIN)


@pytest.mark.parametrize('seed', range(50))
def test_reach_target_is_reachable_and_above_table(seed):
    x, y, z = layout.sample_reach_target(random.Random(seed))
    assert z >= layout.TABLE_TOP + 0.05
    assert is_reachable((x, y, z), layout.MARGIN)


def test_impossible_layout_raises():
    with pytest.raises(RuntimeError):
        layout.sample_layout(random.Random(0), min_separation=1.0, max_attempts=50)


def test_grasp_gap():
    cube = layout.PIECES['red_cube']
    centre = (0.2, 0.0, cube.centre_z)
    touching = (0.2, 0.0, cube.grasp_z)
    assert layout.grasp_gap(cube, centre, touching) == pytest.approx(0.0)
    assert layout.grasp_gap(cube, centre, (0.2, 0.0, cube.grasp_z + 0.004)) == pytest.approx(
        0.004)
    assert layout.grasp_gap(cube, centre, (0.25, 0.0, cube.grasp_z)) == math.inf
```

- [ ] **Step 2: Run to verify failure**

```bash
lab_pytest test/test_layout.py
```

Expected: `ImportError: cannot import name 'layout'`.

- [ ] **Step 3: Implement**

`src/lab_simulation/lab_simulation/layout.py`:

```python
"""
Piece geometry and seeded sampling of layouts and targets.

Pure Python (no ROS). Piece sizes mirror worlds/table_world.sdf; the
consistency test checks them against the file.
"""

import math
import random
from typing import Dict, NamedTuple, Sequence, Tuple

from .kinematics import is_reachable, TIP_RADIUS

TABLE_TOP = 0.75
HOVER = 0.05            # approach height above the grasp point
LIFT = 0.05             # carry height above the grasp point
MARGIN = 0.01           # IK margin around every sampled point
MIN_SEPARATION = 0.08   # piece centre to centre
MIN_RADIUS = 0.12       # clear of the arm base (radius 0.08)
MAX_RADIUS = 0.35       # beyond the arm's reach
MIN_TRAVEL = 0.05       # a place target must differ from the start
GRASP_GAP_MAX = 0.005   # suction engages only within this gap

XY = Tuple[float, float]


class Piece(NamedTuple):
    """A graspable piece: half its height and the grasp footprint half-width."""

    name: str
    half_height: float
    footprint: float

    @property
    def centre_z(self) -> float:
        """Return the centre height of the piece resting on the table."""
        return TABLE_TOP + self.half_height

    @property
    def grasp_z(self) -> float:
        """Return the tip-centre height that touches the top of the resting piece."""
        return TABLE_TOP + 2 * self.half_height + TIP_RADIUS


PIECES: Dict[str, Piece] = {
    piece.name: piece for piece in (
        Piece('red_cube', 0.025, 0.025),
        Piece('blue_cylinder', 0.05, 0.03),
        # A sphere's top is a point: require the tip to be nearly centred.
        Piece('green_sphere', 0.04, 0.01),
    )
}


def grasp_gap(piece: Piece, piece_xyz: Sequence[float], tip_xyz: Sequence[float]) -> float:
    """
    Return the gap between the tip sphere and the top of the piece.

    The gap is infinite when the tip is not above the piece's footprint.
    """
    if math.dist(piece_xyz[:2], tip_xyz[:2]) > piece.footprint:
        return math.inf
    top = piece_xyz[2] + piece.half_height
    return abs((tip_xyz[2] - TIP_RADIUS) - top)


def _sample_xy(rng: random.Random) -> XY:
    r = rng.uniform(MIN_RADIUS, MAX_RADIUS)
    theta = rng.uniform(-3.0, 3.0)
    return (r * math.cos(theta), r * math.sin(theta))


def graspable(piece: Piece, xy: XY) -> bool:
    """Return True if the piece can be grasped and lifted at xy, with margin."""
    return all(
        is_reachable((xy[0], xy[1], z), MARGIN)
        for z in (piece.grasp_z, piece.grasp_z + max(HOVER, LIFT)))


def sample_layout(rng: random.Random, min_separation: float = MIN_SEPARATION,
                  max_attempts: int = 10000) -> Dict[str, XY]:
    """Return a seeded random resting position for every piece."""
    positions: Dict[str, XY] = {}
    for name in sorted(PIECES):
        for _ in range(max_attempts):
            xy = _sample_xy(rng)
            if graspable(PIECES[name], xy) and all(
                    math.dist(xy, other) >= min_separation for other in positions.values()):
                positions[name] = xy
                break
        else:
            raise RuntimeError(f'could not place {name} after {max_attempts} attempts')
    return positions


def sample_place_target(rng: random.Random, name: str, positions: Dict[str, XY],
                        max_attempts: int = 10000) -> XY:
    """Return a seeded random place target for piece name, clear of the others."""
    piece = PIECES[name]
    for _ in range(max_attempts):
        xy = _sample_xy(rng)
        clear = all(
            math.dist(xy, pos) >= (MIN_TRAVEL if other == name else MIN_SEPARATION)
            for other, pos in positions.items())
        if clear and graspable(piece, xy):
            return xy
    raise RuntimeError(f'could not find a place target for {name}')


def sample_reach_target(rng: random.Random,
                        max_attempts: int = 10000) -> Tuple[float, float, float]:
    """Return a seeded random reachable point at least 5 cm above the table."""
    for _ in range(max_attempts):
        x, y = _sample_xy(rng)
        z = rng.uniform(TABLE_TOP + 0.05, TABLE_TOP + 0.65)
        if is_reachable((x, y, z), MARGIN):
            return (x, y, z)
    raise RuntimeError('could not find a reachable target')
```

- [ ] **Step 4: Run the tests**

```bash
lab_pytest test/test_layout.py
```

Expected: 153 passed (most are seed-parametrised).

- [ ] **Step 5: Full suite and commit**

```bash
lab_test
git add src/lab_simulation/lab_simulation/layout.py src/lab_simulation/test/test_layout.py
git commit -m "feat(layout): add piece geometry, grasp gap and seeded layout sampling"
```

---

### Task 5: Metrics, reports and comparison

**Files:**
- Create: `src/lab_simulation/lab_simulation/metrics.py`
- Create: `src/lab_simulation/test/test_metrics.py`

**Interfaces:**
- Consumes: trial dicts with keys `task, index, success, failure, cycle_time_s, error_m, disturbed, details` (the shape of `TrialResult.to_dict()` from Task 6).
- Produces (`lab_simulation.metrics`): `wilson_interval(successes, n, z=1.96) -> (lo, hi)`; `percentile(values, p) -> float | None`; `summarize_task(trials) -> dict`; `summarize(trials, run_info) -> {'run': run_info, 'tasks': {...}}`; `compare(a, b) -> list[row]` where a row has `metric, a, b, delta, ratio, better, significant`; `render_report(summary) -> str`; `render_comparison(rows, label_a, label_b) -> str`.

- [ ] **Step 1: Write the failing tests**

`src/lab_simulation/test/test_metrics.py`:

```python
"""Tests for statistics, summaries, reports and comparisons."""

from lab_simulation import metrics
import pytest


def trial(task='reach', success=True, failure=None, cycle=1.0, error=0.005, disturbed=False):
    return {'task': task, 'index': 0, 'success': success, 'failure': failure,
            'cycle_time_s': cycle, 'error_m': error, 'disturbed': disturbed, 'details': {}}


@pytest.mark.parametrize('k,n,expected', [
    (83, 100, (0.7445, 0.8911)),
    (50, 100, (0.4038, 0.5962)),
    (0, 10, (0.0, 0.2775)),
    (10, 10, (0.7225, 1.0)),
])
def test_wilson_interval(k, n, expected):
    assert metrics.wilson_interval(k, n) == pytest.approx(expected, abs=1e-4)


def test_wilson_interval_with_no_trials():
    assert metrics.wilson_interval(0, 0) == (0.0, 1.0)


def test_percentile():
    assert metrics.percentile([1, 2, 3, 4], 95) == pytest.approx(3.85)
    assert metrics.percentile([1, 2, 3, 4], 50) == pytest.approx(2.5)
    assert metrics.percentile([7], 95) == 7
    assert metrics.percentile([], 95) is None


def test_summarize_task_counts_and_stats():
    trials = [trial(cycle=1.0), trial(cycle=3.0),
              trial(success=False, failure='timeout', cycle=9.0, error=0.05),
              trial(success=False, failure='timeout', error=None, disturbed=True)]
    s = metrics.summarize_task(trials)
    assert (s['trials'], s['successes'], s['success_rate']) == (4, 2, 0.5)
    assert s['cycle_time_s']['mean'] == pytest.approx(2.0)  # successful trials only
    assert s['error_m']['mean'] == pytest.approx((0.005 + 0.005 + 0.05) / 3)
    assert s['failures'] == {'timeout': 2}
    assert s['disturbed'] == 1


def test_summary_with_no_successes_renders():
    summary = metrics.summarize([trial(success=False, failure='grasp_miss', error=None)],
                                {'seed': 1})
    task = summary['tasks']['reach']
    assert task['cycle_time_s'] == {'mean': None, 'median': None, 'p95': None}
    assert 'n/a' in metrics.render_report(summary)


def summary_with(rate_k, n=100, cycle=2.0, rtf=1.0):
    trials = [trial(cycle=cycle) for _ in range(rate_k)] + \
        [trial(success=False, failure='timeout') for _ in range(n - rate_k)]
    return metrics.summarize(trials, {'real_time_factor': rtf, 'trials_per_minute': 10.0})


def test_compare_flags_significance_and_direction():
    rows = {r['metric']: r for r in metrics.compare(summary_with(70), summary_with(95))}
    assert rows['reach success rate']['significant'] is True
    assert rows['reach success rate']['better'] is True
    rows = {r['metric']: r for r in metrics.compare(summary_with(70), summary_with(83))}
    assert rows['reach success rate']['significant'] is False


def test_compare_lower_cycle_time_is_better_and_ratio():
    rows = {r['metric']: r
            for r in metrics.compare(summary_with(90, cycle=4.0), summary_with(90, cycle=1.0))}
    row = rows['reach cycle time mean (s)']
    assert row['better'] is True
    assert row['ratio'] == pytest.approx(0.25)


def test_compare_only_uses_common_tasks():
    a = metrics.summarize([trial()], {})
    b = metrics.summarize([trial(task='pick_place')], {})
    rows = metrics.compare(a, b)
    assert all(not r['metric'].startswith(('reach', 'pick_place')) for r in rows)
    assert 'not significant' not in metrics.render_comparison(rows, 'A', 'B')
```

- [ ] **Step 2: Run to verify failure**

```bash
lab_pytest test/test_metrics.py
```

Expected: `ImportError: cannot import name 'metrics'`.

- [ ] **Step 3: Implement**

`src/lab_simulation/lab_simulation/metrics.py`:

```python
"""
Aggregate trial results into summaries, reports and comparisons.

Pure Python (no ROS). A trial is the dict produced by tasks.TrialResult.
"""

import math
import statistics
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

ERROR_LABELS = {'reach': 'Tip error (m)', 'pick_place': 'Placement error (m)'}


def wilson_interval(successes: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    """Return the Wilson score interval for a success rate; (0, 1) when n is 0."""
    if n == 0:
        return (0.0, 1.0)
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    """Return the p-th percentile of values (p from 0 to 100), or None if empty."""
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * p / 100.0
    low = math.floor(rank)
    high = math.ceil(rank)
    return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


def _stats(values: Sequence[float], keys: Iterable[str]) -> Dict[str, Optional[float]]:
    out: Dict[str, Optional[float]] = {}
    for key in keys:
        if not values:
            out[key] = None
        elif key == 'mean':
            out[key] = statistics.fmean(values)
        elif key == 'median':
            out[key] = statistics.median(values)
        else:
            out[key] = percentile(values, 95)
    return out


def summarize_task(trials: List[dict]) -> dict:
    """Return the aggregate metrics for the trials of one task."""
    n = len(trials)
    successes = sum(1 for t in trials if t['success'])
    failures: Dict[str, int] = {}
    for t in trials:
        if t['failure']:
            failures[t['failure']] = failures.get(t['failure'], 0) + 1
    cycle = [t['cycle_time_s'] for t in trials if t['success']]
    errors = [t['error_m'] for t in trials if t['error_m'] is not None]
    lo, hi = wilson_interval(successes, n)
    return {
        'trials': n,
        'successes': successes,
        'success_rate': successes / n if n else None,
        'success_ci': [lo, hi],
        'cycle_time_s': _stats(cycle, ('mean', 'median', 'p95')),
        'error_m': _stats(errors, ('mean', 'p95')),
        'failures': dict(sorted(failures.items())),
        'disturbed': sum(1 for t in trials if t['disturbed']),
    }


def summarize(trials: List[dict], run_info: dict) -> dict:
    """Return the full summary: run information plus per-task aggregates."""
    tasks = sorted({t['task'] for t in trials})
    return {
        'run': run_info,
        'tasks': {task: summarize_task([t for t in trials if t['task'] == task])
                  for task in tasks},
    }


def _row(metric, a, b, higher_is_better, significant=None):
    delta = None if a is None or b is None else b - a
    ratio = None if a in (None, 0) or b is None else b / a
    better = None
    if delta is not None and delta != 0:
        better = (delta > 0) == higher_is_better
    return {'metric': metric, 'a': a, 'b': b, 'delta': delta, 'ratio': ratio,
            'better': better, 'significant': significant}


def compare(a: dict, b: dict) -> List[dict]:
    """Return metric rows comparing summary a (before) with summary b (after)."""
    rows = []
    for task in sorted(set(a['tasks']) & set(b['tasks'])):
        ta, tb = a['tasks'][task], b['tasks'][task]
        overlap = ta['success_ci'][0] <= tb['success_ci'][1] and \
            tb['success_ci'][0] <= ta['success_ci'][1]
        rows.append(_row(f'{task} success rate', ta['success_rate'], tb['success_rate'],
                         True, significant=not overlap))
        rows.append(_row(f'{task} cycle time mean (s)', ta['cycle_time_s']['mean'],
                         tb['cycle_time_s']['mean'], False))
        rows.append(_row(f'{task} error mean (m)', ta['error_m']['mean'],
                         tb['error_m']['mean'], False))
    for key, label in (('real_time_factor', 'real-time factor'),
                       ('trials_per_minute', 'trials per minute')):
        rows.append(_row(label, a['run'].get(key), b['run'].get(key), True))
    return rows


def _fmt(value: Optional[float], digits: int = 3) -> str:
    return 'n/a' if value is None else f'{value:.{digits}f}'


def render_report(summary: dict) -> str:
    """Return a Markdown report for one summary."""
    run = summary['run']
    lines = [
        '# Benchmark report',
        '',
        f"- Date: {run.get('provenance', {}).get('date', 'unknown')}",
        f"- Commit: {run.get('provenance', {}).get('git_commit', 'unknown')}"
        f" (dirty: {run.get('provenance', {}).get('git_dirty', 'unknown')})",
        f"- Architecture: {run.get('provenance', {}).get('arch', 'unknown')}",
        f"- Seed: {run.get('seed')}, trials per task: {run.get('trials_per_task')}",
        f"- Complete: {run.get('complete')}",
        f"- Real-time factor: {_fmt(run.get('real_time_factor'), 2)},"
        f" trials per minute: {_fmt(run.get('trials_per_minute'), 1)}",
        '',
        '| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Error | Error mean |'
        ' Error p95 | Failures | Disturbed |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for task, s in summary['tasks'].items():
        failures = ', '.join(f'{k}: {v}' for k, v in s['failures'].items()) or 'none'
        rate = 'n/a' if s['success_rate'] is None else f"{100 * s['success_rate']:.0f}%"
        lines.append(
            f"| {task} | {s['successes']}/{s['trials']} ({rate}) |"
            f" {100 * s['success_ci'][0]:.0f}–{100 * s['success_ci'][1]:.0f}% |"
            f" {_fmt(s['cycle_time_s']['mean'], 2)} | {_fmt(s['cycle_time_s']['p95'], 2)} |"
            f" {ERROR_LABELS.get(task, 'Error (m)')} | {_fmt(s['error_m']['mean'], 4)} |"
            f" {_fmt(s['error_m']['p95'], 4)} | {failures} | {s['disturbed']} |")
    return '\n'.join(lines) + '\n'


def render_comparison(rows: List[dict], label_a: str, label_b: str) -> str:
    """Return a Markdown table comparing two runs."""
    lines = [f'| Metric | {label_a} | {label_b} | Change | Ratio | Verdict |',
             '|---|---|---|---|---|---|']
    for row in rows:
        if row['significant'] is False:
            verdict = 'not significant'
        elif row['better'] is None:
            verdict = 'no change'
        else:
            verdict = 'better' if row['better'] else 'worse'
        ratio = 'n/a' if row['ratio'] is None else f"{row['ratio']:.2f}×"
        lines.append(f"| {row['metric']} | {_fmt(row['a'])} | {_fmt(row['b'])} |"
                     f" {_fmt(row['delta'])} | {ratio} | {verdict} |")
    return '\n'.join(lines) + '\n'
```

- [ ] **Step 4: Run the tests**

```bash
lab_pytest test/test_metrics.py
```

Expected: 11 passed.

- [ ] **Step 5: Full suite and commit**

```bash
lab_test
git add src/lab_simulation/lab_simulation/metrics.py src/lab_simulation/test/test_metrics.py
git commit -m "feat(metrics): add success intervals, summaries, reports and run comparison"
```

---

### Task 6: Reach and pick-and-place tasks

**Files:**
- Create: `src/lab_simulation/lab_simulation/tasks.py`
- Create: `src/lab_simulation/test/test_tasks.py`

**Interfaces:**
- Consumes: `kinematics.forward`, `kinematics.inverse`, `kinematics.Unreachable`; `layout` (pieces, sampling, `TIP_RADIUS`, `HOVER`, `LIFT`).
- Consumes (duck-typed; implemented by Task 8): an *arm* with `joint_positions() -> Joints` and `move_to(q, timeout_s, monitor=None) -> MoveResult`; a *scene* with `pose(name) -> (x, y, z)`, `attach(name, tip_xyz) -> bool`, `grasp_gap(name, tip_xyz) -> float`, `detach(name) -> None`, `wait_settled(timeout_s=3.0) -> bool`.
- Produces (`lab_simulation.tasks`): `class SimulationUnresponsive(RuntimeError)`; `class MoveResult(NamedTuple: reached, aborted, settle_time_s, max_error)`; `@dataclass TrialResult(task, index, success, failure, cycle_time_s, error_m=None, disturbed=False, details={})` with `to_dict()`; `ReachTask(arm, now)` and `PickPlaceTask(arm, scene, now)`, each with `name` and `run(rng, index) -> TrialResult`; constants `HOME`, `MOVE_TIMEOUT_S` and the thresholds.

- [ ] **Step 1: Write the failing tests**

`src/lab_simulation/test/test_tasks.py`:

```python
"""Tests for the reach and pick-and-place tasks, using an in-memory fake world."""

import math
import random

from lab_simulation import layout
from lab_simulation.kinematics import forward
from lab_simulation.tasks import MoveResult, PickPlaceTask, ReachTask


class FakeWorld:
    """
    Stand-in for arm.Arm and scene.Scene.

    Moves complete instantly (1 simulated second each). Options inject faults.
    """

    def __init__(self, positions=None, joint_bias=0.0, drop=False, stuck=False,
                 knock_off=False):
        positions = positions or {'red_cube': (0.2, 0.0), 'blue_cylinder': (-0.2, 0.0),
                                  'green_sphere': (0.0, -0.2)}
        self.poses = {n: (x, y, layout.PIECES[n].centre_z) for n, (x, y) in positions.items()}
        self.q = (0.0, 0.0, 0.0)
        self.time = 0.0
        self.attached = None
        self.offset = None
        self.joint_bias = joint_bias
        self.drop = drop
        self.stuck = stuck
        self.knock_off = knock_off

    # arm.Arm interface
    def joint_positions(self):
        return self.q

    def move_to(self, q, timeout_s, monitor=None):
        self.time += 1.0
        if self.stuck:
            return MoveResult(False, False, timeout_s, 1.0)
        self.q = (q[0] + self.joint_bias, q[1], q[2])
        if self.attached and not self.drop:
            tip = forward(self.q)
            self.poses[self.attached] = tuple(t + o for t, o in zip(tip, self.offset))
        if monitor is not None and not monitor():
            return MoveResult(False, True, 0.5, 0.0)
        return MoveResult(True, False, 0.5, 0.0)

    # scene.Scene interface
    def pose(self, name):
        return self.poses[name]

    def grasp_gap(self, name, tip):
        return layout.grasp_gap(layout.PIECES[name], self.poses[name], tip)

    def attach(self, name, tip):
        if self.grasp_gap(name, tip) > layout.GRASP_GAP_MAX:
            return False
        self.attached = name
        self.offset = tuple(p - t for p, t in zip(self.poses[name], tip))
        return True

    def detach(self, name):
        if self.attached == name:
            self.attached = None

    def wait_settled(self, timeout_s=3.0):
        for name, (x, y, _) in self.poses.items():
            self.poses[name] = (x, y, -1.0) if self.knock_off else \
                (x, y, layout.PIECES[name].centre_z)
        return True


def now_of(world):
    return lambda: world.time


def test_reach_success():
    world = FakeWorld()
    result = ReachTask(world, now_of(world)).run(random.Random(1), 0)
    assert result.success and result.failure is None
    assert result.error_m < 1e-9
    assert result.cycle_time_s == 1.0
    assert result.details['max_joint_error_rad'] == 0.0


def test_reach_miss_is_placement_error():
    world = FakeWorld(joint_bias=0.2)
    result = ReachTask(world, now_of(world)).run(random.Random(1), 0)
    assert not result.success and result.failure == 'placement_error'
    assert result.error_m > 0.01


def test_reach_timeout():
    world = FakeWorld(stuck=True)
    result = ReachTask(world, now_of(world)).run(random.Random(1), 0)
    assert result.failure == 'timeout'


def test_pick_place_success_lands_on_target():
    world = FakeWorld()
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.success, result
    assert result.error_m < 1e-6
    assert world.attached is None
    piece = result.details['piece']
    assert math.dist(world.poses[piece][:2], result.details['target']) < 1e-6
    assert result.disturbed is False


def test_pick_place_grasp_miss_never_attaches():
    world = FakeWorld(joint_bias=0.2)
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'grasp_miss'
    assert result.details['grasp_gap_m'] > layout.GRASP_GAP_MAX


def test_pick_place_drop_detected():
    world = FakeWorld(drop=True)
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'dropped'
    assert world.attached is None


def test_pick_place_unreachable_piece():
    world = FakeWorld(positions={'red_cube': (0.6, 0.0), 'blue_cylinder': (-0.6, 0.0),
                                 'green_sphere': (0.0, -0.6)})
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'unreachable'


def test_pick_place_piece_knocked_off_table_is_placement_error():
    world = FakeWorld(knock_off=True)
    result = PickPlaceTask(world, world, now_of(world)).run(random.Random(3), 0)
    assert result.failure == 'placement_error' or result.success
    assert result.details['final'][2] == -1.0
    assert result.disturbed is True


def test_result_serialises():
    world = FakeWorld()
    data = ReachTask(world, now_of(world)).run(random.Random(1), 4).to_dict()
    assert data['task'] == 'reach' and data['index'] == 4
    assert set(data) == {'task', 'index', 'success', 'failure', 'cycle_time_s', 'error_m',
                         'disturbed', 'details'}
```

- [ ] **Step 2: Run to verify failure**

```bash
lab_pytest test/test_tasks.py
```

Expected: `ModuleNotFoundError: No module named 'lab_simulation.tasks'`.

- [ ] **Step 3: Implement**

`src/lab_simulation/lab_simulation/tasks.py`:

```python
"""
Benchmark tasks: reach, and suction pick-and-place.

Pure Python (no ROS). Tasks drive the arm and scene through small interfaces
(arm.Arm and scene.Scene implement them), so they can be unit-tested with
fakes. All times are simulation seconds.
"""

from dataclasses import asdict, dataclass, field
import math
import random
from typing import Callable, Dict, NamedTuple, Optional, Sequence, Tuple

from . import layout
from .kinematics import forward, inverse, Unreachable

REACH_TOLERANCE_M = 0.01
PLACE_TOLERANCE_M = 0.02
CARRY_TOLERANCE_M = 0.01
DISTURB_THRESHOLD_M = 0.01
PLACE_CLEARANCE_M = 0.003
MOVE_TIMEOUT_S = 5.0
HOME = (0.0, 0.0, 0.0)

XYZ = Tuple[float, float, float]


class SimulationUnresponsive(RuntimeError):
    """Raised when the simulation stops publishing feedback."""


class MoveResult(NamedTuple):
    """Outcome of one arm move: settled, aborted by a monitor, timings, error."""

    reached: bool
    aborted: bool
    settle_time_s: float
    max_error: float


@dataclass
class TrialResult:
    """One trial; error_m is tip error (reach) or placement error (pick_place)."""

    task: str
    index: int
    success: bool
    failure: Optional[str]
    cycle_time_s: float
    error_m: Optional[float] = None
    disturbed: bool = False
    details: Dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Return the result as a JSON-serialisable dict."""
        return asdict(self)


def _add(a: Sequence[float], b: Sequence[float]) -> XYZ:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Sequence[float], b: Sequence[float]) -> XYZ:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


class ReachTask:
    """Move the tip to a random reachable point and measure the error."""

    name = 'reach'

    def __init__(self, arm, now: Callable[[], float]):
        self._arm = arm
        self._now = now

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """Run one reach trial."""
        target = layout.sample_reach_target(rng)
        details = {'target': list(target)}
        start = self._now()
        try:
            q = inverse(target)
        except Unreachable:
            return TrialResult(self.name, index, False, 'unreachable', 0.0, details=details)
        move = self._arm.move_to(q, MOVE_TIMEOUT_S)
        elapsed = self._now() - start
        error = math.dist(forward(self._arm.joint_positions()), target)
        details['settle_time_s'] = move.settle_time_s
        details['max_joint_error_rad'] = move.max_error
        if not move.reached:
            return TrialResult(self.name, index, False, 'timeout', elapsed, error,
                               details=details)
        ok = error <= REACH_TOLERANCE_M
        return TrialResult(self.name, index, ok, None if ok else 'placement_error',
                           elapsed, error, details=details)


class PickPlaceTask:
    """Pick a random piece with suction and place it at a random target."""

    name = 'pick_place'

    def __init__(self, arm, scene, now: Callable[[], float]):
        self._arm = arm
        self._scene = scene
        self._now = now

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """Run one pick-and-place trial from the current scene."""
        names = sorted(layout.PIECES)
        start = {name: self._scene.pose(name) for name in names}
        name = rng.choice(names)
        target = layout.sample_place_target(
            rng, name, {n: (p[0], p[1]) for n, p in start.items()})
        details = {'piece': name, 'start': list(start[name]), 'target': list(target)}
        t0 = self._now()
        try:
            failure = self._execute(layout.PIECES[name], start[name], target, details)
        finally:
            self._scene.detach(name)
        elapsed = self._now() - t0
        if failure is not None:
            return TrialResult(self.name, index, False, failure, elapsed, details=details)
        self._scene.wait_settled()
        final = self._scene.pose(name)
        details['final'] = list(final)
        error = math.dist(final[:2], target)
        disturbed = any(
            math.dist(self._scene.pose(other), start[other]) > DISTURB_THRESHOLD_M
            for other in names if other != name)
        ok = error <= PLACE_TOLERANCE_M
        return TrialResult(self.name, index, ok, None if ok else 'placement_error',
                           elapsed, error, disturbed, details)

    def _move(self, xyz: XYZ, monitor=None) -> Optional[str]:
        try:
            q = inverse(xyz)
        except Unreachable:
            return 'unreachable'
        result = self._arm.move_to(q, MOVE_TIMEOUT_S, monitor)
        if result.aborted:
            return 'dropped'
        if not result.reached:
            return 'timeout'
        return None

    def _tip(self) -> XYZ:
        return forward(self._arm.joint_positions())

    def _execute(self, piece, start: XYZ, target, details: dict) -> Optional[str]:
        x, y, z = start
        grasp_z = z + piece.half_height + layout.TIP_RADIUS
        for xyz in ((x, y, grasp_z + layout.HOVER), (x, y, grasp_z)):
            failure = self._move(xyz)
            if failure:
                return failure

        tip = self._tip()
        if not self._scene.attach(piece.name, tip):
            details['grasp_gap_m'] = self._scene.grasp_gap(piece.name, tip)
            return 'grasp_miss'
        offset = _sub(self._scene.pose(piece.name), tip)

        def carrying() -> bool:
            expected = _add(self._tip(), offset)
            return math.dist(self._scene.pose(piece.name), expected) <= CARRY_TOLERANCE_M

        # Aim the tip so that the piece, not the tip, lands on the target.
        tx, ty = target[0] - offset[0], target[1] - offset[1]
        place_z = piece.centre_z - offset[2] + PLACE_CLEARANCE_M
        for xyz in ((tip[0], tip[1], tip[2] + layout.LIFT),
                    (tx, ty, place_z + layout.LIFT),
                    (tx, ty, place_z)):
            failure = self._move(xyz, carrying)
            if failure:
                return failure

        self._scene.detach(piece.name)
        failure = self._move((tx, ty, place_z + layout.HOVER))
        if failure:
            return failure
        home = self._arm.move_to(HOME, MOVE_TIMEOUT_S)
        return None if home.reached else 'timeout'
```

- [ ] **Step 4: Run the tests**

```bash
lab_pytest test/test_tasks.py
```

Expected: 9 passed.

- [ ] **Step 5: Full suite and commit**

```bash
lab_test
git add src/lab_simulation/lab_simulation/tasks.py src/lab_simulation/test/test_tasks.py
git commit -m "feat(tasks): add reach and suction pick-and-place trials with failure reasons"
```

---

### Task 7: World, suction and bridge

**Files:**
- Modify: `src/lab_simulation/worlds/table_world.sdf`
- Modify: `src/lab_simulation/config/bridge.yaml`
- Modify: `src/lab_simulation/test/test_robot_description.py`

**Interfaces:**
- Consumes: Task 1 answers (suction and `set_pose` work; the combined pose stream loses names, so each piece publishes its own pose — approved by the user); `layout.PIECES`, `layout.graspable`.
- Produces: ROS topics `/lab/pieces/<piece>/pose` (`geometry_msgs/msg/PoseStamped`, world frame), `/simple_arm/suction/<piece>/attach` and `/detach` (`std_msgs/msg/Empty`), service `/world/table_world/set_pose` (`ros_gz_interfaces/srv/SetEntityPose`).

- [ ] **Step 1: Write the failing tests**

In `src/lab_simulation/test/test_robot_description.py`:

1. Below `import pytest`, add `from lab_simulation import layout` (keep it in the alphabetically sorted third-party block: it sorts before `pytest`, so place it directly above `import pytest`), and after the `BRIDGE = ...` line add:

```python
WORLD_MODELS = {m.get('name'): m for m in WORLD.findall('model')}
DETACHABLE = 'gz::sim::systems::DetachableJoint'
```

2. Service entries have no `direction` or `ros_topic_name`. In `test_controller_topics_are_bridged`, change `if e['direction'] == 'ROS_TO_GZ'` to `if e.get('direction') == 'ROS_TO_GZ'`; in `test_joint_states_bridged_from_world`, change `if e['ros_topic_name'] == '/joint_states'` to `if e.get('ros_topic_name') == '/joint_states'`.

3. Append:

```python
def arm_include():
    return next(i for i in WORLD.findall('include') if i.find('name').text == 'simple_arm')


def test_suction_joints_reference_real_entities():
    plugins = [p for p in arm_include().findall('plugin') if p.get('name') == DETACHABLE]
    assert sorted(p.find('child_model').text for p in plugins) == sorted(layout.PIECES)
    arm_links = {link.get('name') for link in SDF_MODEL.findall('link')}
    for plugin in plugins:
        name = plugin.find('child_model').text
        assert plugin.find('parent_link').text in arm_links
        piece_links = {link.get('name') for link in WORLD_MODELS[name].findall('link')}
        assert plugin.find('child_link').text in piece_links
        assert plugin.find('attach_topic').text == f'/simple_arm/suction/{name}/attach'
        assert plugin.find('detach_topic').text == f'/simple_arm/suction/{name}/detach'


def test_suction_topics_are_bridged():
    ros_to_gz = {(e['ros_topic_name'], e['gz_topic_name'], e['ros_type_name'], e['gz_type_name'])
                 for e in BRIDGE if e.get('direction') == 'ROS_TO_GZ'}
    for name in layout.PIECES:
        for action in ('attach', 'detach'):
            topic = f'/simple_arm/suction/{name}/{action}'
            assert (topic, topic, 'std_msgs/msg/Empty', 'gz.msgs.Empty') in ros_to_gz


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_piece_pose_is_published_and_bridged(name):
    plugin = next(p for p in WORLD_MODELS[name].findall('plugin')
                  if p.get('name') == 'gz::sim::systems::PosePublisher')
    assert plugin.find('publish_model_pose').text == 'true'
    assert plugin.find('use_pose_vector_msg').text == 'false'
    entry = next(e for e in BRIDGE if e.get('ros_topic_name') == f'/lab/pieces/{name}/pose')
    assert entry['gz_topic_name'] == f'/model/{name}/pose'
    assert (entry['ros_type_name'], entry['gz_type_name']) == (
        'geometry_msgs/msg/PoseStamped', 'gz.msgs.Pose')
    assert entry['direction'] == 'GZ_TO_ROS'


def test_reset_service_is_bridged():
    service = next(e for e in BRIDGE if e.get('service_name') == '/world/table_world/set_pose')
    assert service['ros_type_name'] == 'ros_gz_interfaces/srv/SetEntityPose'
    assert (service['gz_req_type_name'], service['gz_rep_type_name']) == (
        'gz.msgs.Pose', 'gz.msgs.Boolean')


@pytest.mark.parametrize('name', sorted(layout.PIECES))
def test_piece_geometry_matches_layout(name):
    piece = layout.PIECES[name]
    model = WORLD_MODELS[name]
    geometry = model.find('link/collision/geometry')[0]
    if geometry.tag == 'box':
        half = floats(geometry.find('size').text)[2] / 2
    elif geometry.tag == 'cylinder':
        half = float(geometry.find('length').text) / 2
    else:
        half = float(geometry.find('radius').text)
    assert piece.half_height == pytest.approx(half)
    x, y, z = sdf_xyz(model)
    assert z == pytest.approx(piece.centre_z)
    assert layout.graspable(piece, (x, y)), f'{name} default position is out of reach'
```

- [ ] **Step 2: Run to verify failure**

```bash
lab_pytest test/test_robot_description.py
```

Expected: the new tests fail (no plugins, no bridge entries, pieces out of reach); existing tests pass.

- [ ] **Step 3: Move the pieces inside reach**

In `src/lab_simulation/worlds/table_world.sdf`, change the three piece poses:

| Model | Old `<pose>` | New `<pose>` |
|---|---|---|
| `red_cube` | `0.25 0.15 0.775 0 0 0` | `0.173 0.10 0.775 0 0 0` |
| `blue_cylinder` | `-0.20 0.20 0.80 0 0 0` | `-0.173 0.10 0.80 0 0 0` |
| `green_sphere` | `0.10 -0.25 0.79 0 0 0` | `0.0 -0.20 0.79 0 0 0` |

Each is 0.20 m from the base axis. In the "OBJECTS ON THE TABLE" comment, add: `Default positions are 0.20 m from the base axis, inside the arm's reach.`

- [ ] **Step 4: Publish each piece's pose**

Gazebo's combined pose stream loses entity names when bridged (Task 1), so each piece publishes its own pose. In each of the three piece models (`red_cube`, `blue_cylinder`, `green_sphere`), add directly before its closing `</model>`:

```xml
      <!-- Publishes this piece's world pose on /model/<name>/pose. -->
      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_model_pose>true</publish_model_pose>
        <publish_link_pose>false</publish_link_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>50</update_frequency>
      </plugin>
```

- [ ] **Step 4b: Add the suction plugins to the arm include**

Replace the arm `<include>` element with:

```xml
    <include>
      <uri>model://simple_arm.sdf</uri>
      <name>simple_arm</name>
      <pose>0 0 0.75 0 0 0</pose>

      <!--
        Simulated suction: one DetachableJoint per piece, fixing the piece to
        the end effector at its current offset. Declared here rather than in
        simple_arm.sdf so the arm model does not name this world's pieces.
        lab_simulation.scene engages suction only when the tip is within 5 mm
        of the piece (layout.GRASP_GAP_MAX).
      -->
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>end_effector</parent_link>
        <child_model>red_cube</child_model>
        <child_link>cube_link</child_link>
        <attach_topic>/simple_arm/suction/red_cube/attach</attach_topic>
        <detach_topic>/simple_arm/suction/red_cube/detach</detach_topic>
        <suppress_child_warning>true</suppress_child_warning>
      </plugin>
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>end_effector</parent_link>
        <child_model>blue_cylinder</child_model>
        <child_link>cylinder_link</child_link>
        <attach_topic>/simple_arm/suction/blue_cylinder/attach</attach_topic>
        <detach_topic>/simple_arm/suction/blue_cylinder/detach</detach_topic>
        <suppress_child_warning>true</suppress_child_warning>
      </plugin>
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>end_effector</parent_link>
        <child_model>green_sphere</child_model>
        <child_link>sphere_link</child_link>
        <attach_topic>/simple_arm/suction/green_sphere/attach</attach_topic>
        <detach_topic>/simple_arm/suction/green_sphere/detach</detach_topic>
        <suppress_child_warning>true</suppress_child_warning>
      </plugin>
    </include>
```

- [ ] **Step 5: Extend the bridge**

Append to `src/lab_simulation/config/bridge.yaml`:

```yaml

# Live world pose of each piece, from its PosePublisher (one topic per piece:
# the combined pose stream loses entity names when bridged).
- ros_topic_name: /lab/pieces/red_cube/pose
  gz_topic_name: /model/red_cube/pose
  ros_type_name: geometry_msgs/msg/PoseStamped
  gz_type_name: gz.msgs.Pose
  direction: GZ_TO_ROS

- ros_topic_name: /lab/pieces/blue_cylinder/pose
  gz_topic_name: /model/blue_cylinder/pose
  ros_type_name: geometry_msgs/msg/PoseStamped
  gz_type_name: gz.msgs.Pose
  direction: GZ_TO_ROS

- ros_topic_name: /lab/pieces/green_sphere/pose
  gz_topic_name: /model/green_sphere/pose
  ros_type_name: geometry_msgs/msg/PoseStamped
  gz_type_name: gz.msgs.Pose
  direction: GZ_TO_ROS

# Suction: engage or release one piece (see table_world.sdf).
- ros_topic_name: /simple_arm/suction/red_cube/attach
  gz_topic_name: /simple_arm/suction/red_cube/attach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /simple_arm/suction/red_cube/detach
  gz_topic_name: /simple_arm/suction/red_cube/detach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /simple_arm/suction/blue_cylinder/attach
  gz_topic_name: /simple_arm/suction/blue_cylinder/attach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /simple_arm/suction/blue_cylinder/detach
  gz_topic_name: /simple_arm/suction/blue_cylinder/detach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /simple_arm/suction/green_sphere/attach
  gz_topic_name: /simple_arm/suction/green_sphere/attach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /simple_arm/suction/green_sphere/detach
  gz_topic_name: /simple_arm/suction/green_sphere/detach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

# Scene reset: teleport a model (benchmark only).
- service_name: /world/table_world/set_pose
  ros_type_name: ros_gz_interfaces/srv/SetEntityPose
  gz_req_type_name: gz.msgs.Pose
  gz_rep_type_name: gz.msgs.Boolean
```

- [ ] **Step 6: Run the tests**

```bash
lab_pytest test/test_robot_description.py
```

Expected: all pass.

- [ ] **Step 7: Live check**

```bash
docker exec -d ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 launch lab_simulation arm_world.launch.py rviz:=false > /tmp/launch.log 2>&1'
sleep 20
docker exec ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 topic list | grep -E "lab/pieces|suction" && ros2 service list | grep set_pose && timeout 15 ros2 topic echo --once /lab/pieces/red_cube/pose geometry_msgs/msg/PoseStamped | grep -A3 position'
```

Expected: three `/lab/pieces/<piece>/pose` topics, six `/simple_arm/suction/...` topics, `/world/table_world/set_pose`, and a red-cube position near (0.173, 0.10, 0.775). Stop the simulation with `./stop-lab.sh && ./start-lab.sh` (simplest clean stop).

- [ ] **Step 8: Full suite and commit**

```bash
lab_test
git add src/lab_simulation/worlds/table_world.sdf src/lab_simulation/config/bridge.yaml src/lab_simulation/test/test_robot_description.py
git commit -m "feat(sim): add suction joints, per-piece poses and scene reset to the world and bridge

Pieces move inside the arm's reach. The robot-description tests now
tolerate service entries in the bridge configuration."
```

---

### Task 8: ROS adapters: arm and scene

**Files:**
- Create: `src/lab_simulation/lab_simulation/ros_util.py`, `src/lab_simulation/lab_simulation/arm.py`, `src/lab_simulation/lab_simulation/scene.py`
- Modify: `src/lab_simulation/package.xml`

**Interfaces:**
- Consumes: `tasks.MoveResult`, `tasks.SimulationUnresponsive`, `kinematics.Joints`, `layout`; ROS interfaces from Task 7 (`/lab/pieces/<piece>/pose`, suction topics, `set_pose`).
- Produces: `scene.POSE_TOPIC = '/lab/pieces/{name}/pose'`; `ros_util.sim_now(node) -> float`, `ros_util.spin_until(node, predicate, sim_timeout_s, wall_timeout_s=60.0) -> bool` (raises `SimulationUnresponsive`), `ros_util.spin_for(node, sim_seconds)`; `arm.Arm(node)` implementing the *arm* interface of Task 6 plus `wall_seconds_since_feedback()`; `scene.Scene(node)` implementing the *scene* interface of Task 6 plus `has_poses()`, `detach_all()`, `reset(positions, timeout_s=3.0) -> bool`.

These modules are thin adapters over ROS and are verified against the live simulation (Step 4) and end to end by the benchmark smoke run in Task 10.

- [ ] **Step 1: Add runtime dependencies**

In `src/lab_simulation/package.xml`, add (keeping the `exec_depend` list alphabetical):

```xml
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>ros_gz_interfaces</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
```

- [ ] **Step 2: Implement the adapters**

`src/lab_simulation/lab_simulation/ros_util.py`:

```python
"""Small helpers for spinning a node against simulation time."""

import time
from typing import Callable

import rclpy
from rclpy.node import Node

from .tasks import SimulationUnresponsive


def sim_now(node: Node) -> float:
    """Return the node's current time in seconds (simulation time with use_sim_time)."""
    return node.get_clock().now().nanoseconds / 1e9


def spin_until(node: Node, predicate: Callable[[], bool], sim_timeout_s: float,
               wall_timeout_s: float = 60.0) -> bool:
    """
    Spin node until predicate() is true.

    Returns False if sim_timeout_s of simulation time passes first. Raises
    SimulationUnresponsive if wall_timeout_s of wall time passes first, which
    means the simulation has stalled.
    """
    sim_start = sim_now(node)
    wall_start = time.monotonic()
    while not predicate():
        if sim_now(node) - sim_start >= sim_timeout_s:
            return False
        if time.monotonic() - wall_start >= wall_timeout_s:
            raise SimulationUnresponsive(f'no progress for {wall_timeout_s:.0f} s of wall time')
        rclpy.spin_once(node, timeout_sec=0.01)
    return True


def spin_for(node: Node, sim_seconds: float) -> None:
    """Spin node for sim_seconds of simulation time."""
    spin_until(node, lambda: False, sim_seconds)
```

`src/lab_simulation/lab_simulation/arm.py`:

```python
"""Command the simulated arm's joints and wait for it to settle."""

import time
from typing import Callable, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .kinematics import Joints
from .ros_util import sim_now
from .tasks import MoveResult, SimulationUnresponsive

JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3')
REPUBLISH_S = 0.1       # resend targets in case a message was lost
MIN_MOVE_S = 0.3        # do not judge "settled" before motion has started
STILL_SPEED = 0.02      # rad/s: below this a joint counts as still
HOLD_S = 0.3            # joints must stay still this long to count as settled
FEEDBACK_TIMEOUT_S = 5.0


class Arm:
    """Joint position interface to the Gazebo controllers through the bridge."""

    def __init__(self, node: Node):
        self._node = node
        self._publishers = {
            name: node.create_publisher(Float64, f'/simple_arm/{name}/cmd_pos', 10)
            for name in JOINT_NAMES
        }
        self._positions = {}
        self._velocities = {}
        self._last_feedback = None
        node.create_subscription(
            JointState, '/joint_states', self._on_joint_states, qos_profile_sensor_data)

    def _on_joint_states(self, msg: JointState) -> None:
        self._positions.update(zip(msg.name, msg.position))
        self._velocities.update(zip(msg.name, msg.velocity))
        self._last_feedback = time.monotonic()

    def joint_positions(self) -> Optional[Joints]:
        """Return the latest measured joint angles, or None before feedback arrives."""
        if not all(name in self._positions for name in JOINT_NAMES):
            return None
        return Joints(*(self._positions[name] for name in JOINT_NAMES))

    def wall_seconds_since_feedback(self) -> float:
        """Return wall seconds since the last /joint_states message."""
        if self._last_feedback is None:
            return float('inf')
        return time.monotonic() - self._last_feedback

    def _publish(self, q: Sequence[float]) -> None:
        for name, value in zip(JOINT_NAMES, q):
            self._publishers[name].publish(Float64(data=float(value)))

    def _max_error(self, q: Sequence[float]) -> float:
        current = self.joint_positions()
        return max(abs(a - b) for a, b in zip(current, q)) if current else float('inf')

    def _max_speed(self) -> float:
        return max((abs(self._velocities.get(name, 0.0)) for name in JOINT_NAMES), default=0.0)

    def move_to(self, q: Sequence[float], timeout_s: float,
                monitor: Optional[Callable[[], bool]] = None) -> MoveResult:
        """
        Command joint angles q and wait until the arm has settled.

        Settled means every joint is slower than STILL_SPEED for HOLD_S, judged
        only after MIN_MOVE_S. A monitor returning False aborts the move.
        Times are simulation seconds.
        """
        start = sim_now(self._node)
        last_publish = None
        still_since = None
        while True:
            now = sim_now(self._node)
            if last_publish is None or now - last_publish >= REPUBLISH_S:
                self._publish(q)
                last_publish = now
            if self.wall_seconds_since_feedback() > FEEDBACK_TIMEOUT_S:
                raise SimulationUnresponsive(
                    f'no /joint_states for {FEEDBACK_TIMEOUT_S:.0f} s of wall time')
            if monitor is not None and not monitor():
                return MoveResult(False, True, now - start, self._max_error(q))
            if now - start >= MIN_MOVE_S and self._max_speed() < STILL_SPEED:
                still_since = now if still_since is None else still_since
                if now - still_since >= HOLD_S:
                    return MoveResult(True, False, still_since - start, self._max_error(q))
            else:
                still_since = None
            if now - start >= timeout_s:
                return MoveResult(False, False, now - start, self._max_error(q))
            rclpy.spin_once(self._node, timeout_sec=0.01)
```

`src/lab_simulation/lab_simulation/scene.py`:

```python
"""Piece poses, scene reset and suction, through the Gazebo bridge."""

from functools import partial
import math
from typing import Dict, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Empty

from . import layout
from .ros_util import spin_for, spin_until

# One pose topic per piece, bridged from each piece's PosePublisher.
POSE_TOPIC = '/lab/pieces/{name}/pose'
SET_POSE_SERVICE = '/world/table_world/set_pose'
SETTLE_SPEED = 0.001    # m/s: below this a piece counts as at rest
SETTLE_SAMPLE_S = 0.1

XYZ = Tuple[float, float, float]


class Scene:
    """Read piece poses, reset the layout, and engage or release suction."""

    def __init__(self, node: Node):
        self._node = node
        self._poses: Dict[str, XYZ] = {}
        for name in layout.PIECES:
            node.create_subscription(PoseStamped, POSE_TOPIC.format(name=name),
                                     partial(self._on_pose, name), qos_profile_sensor_data)
        self._set_pose = node.create_client(SetEntityPose, SET_POSE_SERVICE)
        self._attach = {n: node.create_publisher(Empty, f'/simple_arm/suction/{n}/attach', 10)
                        for n in layout.PIECES}
        self._detach = {n: node.create_publisher(Empty, f'/simple_arm/suction/{n}/detach', 10)
                        for n in layout.PIECES}

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._poses[name] = (p.x, p.y, p.z)

    def has_poses(self) -> bool:
        """Return True once every piece's pose has been received."""
        return all(name in self._poses for name in layout.PIECES)

    def pose(self, name: str) -> Optional[XYZ]:
        """Return the latest world position of a piece's centre."""
        return self._poses.get(name)

    def grasp_gap(self, name: str, tip_xyz: Sequence[float]) -> float:
        """Return the measured gap between the tip sphere and the top of a piece."""
        return layout.grasp_gap(layout.PIECES[name], self._poses[name], tip_xyz)

    def attach(self, name: str, tip_xyz: Sequence[float]) -> bool:
        """Engage suction on a piece if the tip is within GRASP_GAP_MAX of it."""
        if self.grasp_gap(name, tip_xyz) > layout.GRASP_GAP_MAX:
            return False
        self._attach[name].publish(Empty())
        spin_for(self._node, 0.05)
        return True

    def detach(self, name: str) -> None:
        """Release suction on a piece."""
        self._detach[name].publish(Empty())
        spin_for(self._node, 0.05)

    def detach_all(self) -> None:
        """Release suction on every piece."""
        for name in layout.PIECES:
            self._detach[name].publish(Empty())
        spin_for(self._node, 0.05)

    def _place(self, name: str, x: float, y: float) -> bool:
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose = Pose()
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = layout.PIECES[name].centre_z
        request.pose.orientation.w = 1.0
        if not self._set_pose.wait_for_service(timeout_sec=5.0):
            return False
        future = self._set_pose.call_async(request)
        spin_until(self._node, future.done, sim_timeout_s=10.0)
        return future.done() and future.result() is not None and future.result().success

    def reset(self, positions: Dict[str, Tuple[float, float]], timeout_s: float = 3.0) -> bool:
        """Teleport every piece to positions, upright, and wait until they rest."""
        for name, (x, y) in positions.items():
            if not self._place(name, x, y):
                return False
        return self.wait_settled(timeout_s)

    def wait_settled(self, timeout_s: float = 3.0) -> bool:
        """Wait until every piece moves slower than SETTLE_SPEED."""
        waited = 0.0
        previous = dict(self._poses)
        while waited < timeout_s:
            spin_for(self._node, SETTLE_SAMPLE_S)
            waited += SETTLE_SAMPLE_S
            current = dict(self._poses)
            if all(math.dist(current[n], previous[n]) <= SETTLE_SPEED * SETTLE_SAMPLE_S
                   for n in layout.PIECES if n in current and n in previous):
                return True
            previous = current
        return False
```

- [ ] **Step 3: Build and lint**

```bash
lab_test
```

Expected: 0 failures (flake8 and pep257 cover the new modules).

- [ ] **Step 4: Live check against the simulation**

Create `/tmp/check_adapters.py` on the host:

```python
"""Throwaway live check for Task 8: one move, one reset, one grasp attempt."""
import random

import rclpy
from rclpy.parameter import Parameter

from lab_simulation import layout
from lab_simulation.arm import Arm
from lab_simulation.kinematics import forward, inverse
from lab_simulation.ros_util import sim_now, spin_until
from lab_simulation.scene import Scene

rclpy.init()
node = rclpy.create_node('check', parameter_overrides=[Parameter('use_sim_time', value=True)])
arm, scene = Arm(node), Scene(node)
spin_until(node, lambda: sim_now(node) > 0 and arm.joint_positions() and scene.has_poses(),
           float('inf'))
print('poses:', {n: tuple(round(v, 3) for v in scene.pose(n)) for n in layout.PIECES})
print('home:', arm.move_to((0.0, 0.0, 0.0), 5.0))
print('reset:', scene.reset(layout.sample_layout(random.Random(1))))
cube = scene.pose('red_cube')
grasp = (cube[0], cube[1], cube[2] + layout.PIECES['red_cube'].half_height + layout.TIP_RADIUS)
print('descend:', arm.move_to(inverse(grasp), 5.0))
tip = forward(arm.joint_positions())
print('gap (m):', round(scene.grasp_gap('red_cube', tip), 4), 'attach:', scene.attach('red_cube', tip))
scene.detach_all()
```

Run it with the simulation up:

```bash
docker exec -d ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 launch lab_simulation arm_world.launch.py rviz:=false > /tmp/launch.log 2>&1'
sleep 20
docker cp /tmp/check_adapters.py ai-robotics-lab:/tmp/check_adapters.py
docker exec ai-robotics-lab bash -c 'source /workspace/install/setup.bash && python3 /tmp/check_adapters.py'
```

Expected: three piece poses printed; `home` and `descend` show `reached=True`; `reset: True`; a gap value printed. `attach` is `True` if the gap is ≤ 0.005 m. A larger gap is acceptable here (it is measured, not hidden) but write the value down: Task 10's smoke run shows whether it is systematic. Then `./stop-lab.sh && ./start-lab.sh`.

- [ ] **Step 5: Commit**

```bash
git add src/lab_simulation/package.xml src/lab_simulation/lab_simulation/ros_util.py src/lab_simulation/lab_simulation/arm.py src/lab_simulation/lab_simulation/scene.py
git commit -m "feat(sim): add ROS adapters for arm motion, piece poses, reset and suction"
```

---

### Task 9: See the pieces in RViz

**Files:**
- Create: `src/lab_simulation/lab_simulation/object_markers.py`, `src/lab_simulation/scripts/object_markers.py`
- Modify: `src/lab_simulation/CMakeLists.txt`, `src/lab_simulation/config/arm.rviz`, `src/lab_simulation/launch/arm_world.launch.py`

**Interfaces:**
- Consumes: `scene.POSE_TOPIC` per-piece poses; `layout.PIECES`; Task 1's `ATTACHED_AT_START=yes` (suction attaches at start-up, so the launch file releases it).
- Produces: `visualization_msgs/msg/MarkerArray` on `/lab/piece_markers`; executable `object_markers.py`.

- [ ] **Step 1: Implement the node and entry point**

`src/lab_simulation/lab_simulation/object_markers.py`:

```python
"""Publish RViz markers for the table and the pieces at their live poses."""

from functools import partial
from typing import Dict

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from . import layout
from .scene import POSE_TOPIC

TOPIC = '/lab/piece_markers'

# (marker type, scale x, y, z, colour r, g, b) mirroring table_world.sdf.
SHAPES = {
    'red_cube': (Marker.CUBE, 0.05, 0.05, 0.05, 0.8, 0.1, 0.1),
    'blue_cylinder': (Marker.CYLINDER, 0.06, 0.06, 0.10, 0.1, 0.1, 0.8),
    'green_sphere': (Marker.SPHERE, 0.08, 0.08, 0.08, 0.1, 0.7, 0.2),
}


class ObjectMarkers(Node):
    """Republish piece poses from Gazebo as a MarkerArray, plus the table."""

    def __init__(self):
        super().__init__('object_markers')
        self._poses: Dict[str, Pose] = {}
        self._publisher = self.create_publisher(MarkerArray, TOPIC, 10)
        for name in layout.PIECES:
            self.create_subscription(PoseStamped, POSE_TOPIC.format(name=name),
                                     partial(self._on_pose, name), qos_profile_sensor_data)
        self.create_timer(0.1, self._publish)

    def _on_pose(self, name: str, msg: PoseStamped) -> None:
        self._poses[name] = msg.pose

    def _marker(self, marker_id, kind, scale, colour):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lab'
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = colour
        marker.color.a = 1.0
        return marker

    def _publish(self) -> None:
        markers = MarkerArray()
        table = self._marker(0, Marker.CUBE, (1.0, 1.0, 0.05), (0.6, 0.4, 0.2))
        table.pose.position.z = 0.725
        table.pose.orientation.w = 1.0
        table.color.a = 0.6
        markers.markers.append(table)
        for marker_id, name in enumerate(sorted(SHAPES), start=1):
            if name not in self._poses:
                continue
            kind, sx, sy, sz, r, g, b = SHAPES[name]
            marker = self._marker(marker_id, kind, (sx, sy, sz), (r, g, b))
            marker.pose = self._poses[name]
            markers.markers.append(marker)
        self._publisher.publish(markers)


def main() -> None:
    """Run the marker publisher until shutdown."""
    rclpy.init()
    node = ObjectMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

`src/lab_simulation/scripts/object_markers.py` (make it executable: `chmod +x`):

```python
#!/usr/bin/env python3
"""Entry point for lab_simulation.object_markers (RViz markers for the pieces)."""

from lab_simulation.object_markers import main

if __name__ == '__main__':
    main()
```

In `src/lab_simulation/CMakeLists.txt`, add `scripts/object_markers.py` to the `install(PROGRAMS ...)` list.

- [ ] **Step 2: Add the RViz display**

In `src/lab_simulation/config/arm.rviz`, insert directly above the line `  Enabled: true` that precedes `  Global Options:`:

```yaml
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Pieces
      Namespaces:
        {}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /lab/piece_markers
      Value: true
```

- [ ] **Step 3: Start the markers (and release suction) from the launch file**

In `src/lab_simulation/launch/arm_world.launch.py`:
- Add imports, one per line, keeping the block sorted: `from lab_simulation.layout import PIECES` directly after the `ament_index_python` import (it sorts before `launch`), and `from launch.actions import ExecuteProcess` and `from launch.actions import TimerAction` among the existing `launch.actions` imports in alphabetical order.
- Before `return LaunchDescription([`, add:

```python
    object_markers = Node(
        package='lab_simulation',
        executable='object_markers.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
```

- Task 1 recorded `ATTACHED_AT_START=yes` and that `-p ''` works, so also add:

```python
    # DetachableJoint attaches at start-up; release every piece once Gazebo runs.
    release_suction = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['gz', 'topic', '-t', f'/simple_arm/suction/{name}/detach',
                 '-m', 'gz.msgs.Empty', '-p', ''],
            output='log')
        for name in sorted(PIECES)])
```

and add `release_suction` to the returned list.
- Add `object_markers` to the returned `LaunchDescription` list, after `rviz`.
- In the module docstring's list, add `5. Piece markers for RViz (object_markers)`.

- [ ] **Step 4: Build, lint, test**

```bash
lab_test
```

Expected: 0 failures.

- [ ] **Step 5: Visual check**

```bash
docker exec -d ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 launch lab_simulation arm_world.launch.py > /tmp/launch.log 2>&1'
```

Open `http://localhost:6080`. Expected: RViz shows the arm, a translucent brown table, and the red cube, blue cylinder and green sphere at their new positions around the base. Then run `docker cp /tmp/check_adapters.py ai-robotics-lab:/tmp/ && docker exec ai-robotics-lab bash -c 'source /workspace/install/setup.bash && python3 /tmp/check_adapters.py'` and watch the pieces jump to a new layout and the arm descend onto the red cube. Stop with `./stop-lab.sh && ./start-lab.sh`.

- [ ] **Step 6: Commit**

```bash
git add src/lab_simulation/lab_simulation/object_markers.py src/lab_simulation/scripts/object_markers.py src/lab_simulation/CMakeLists.txt src/lab_simulation/config/arm.rviz src/lab_simulation/launch/arm_world.launch.py
git commit -m "feat(viz): show the table and pieces in RViz at their live poses"
```

---

### Task 10: Benchmark runner

**Files:**
- Create: `src/lab_simulation/lab_simulation/benchmark.py`, `src/lab_simulation/scripts/benchmark.py`, `src/lab_simulation/launch/benchmark.launch.py`, `src/lab_simulation/test/test_benchmark.py`, `run-benchmark.sh`
- Modify: `src/lab_simulation/CMakeLists.txt`, `docker-compose.yml`, `.gitignore`

**Interfaces:**
- Consumes: everything from Tasks 3–9.
- Produces (`lab_simulation.benchmark`): `make_run_dir(base, now) -> Path`; `execute(tasks, trials_per_task, seed, reset, trials_path) -> (trials, complete)`; `provenance() -> dict`; `write_outputs(run_dir, trials, run_info) -> summary`; `compare_dirs(a, b) -> str`; `set_real_time_factor(value) -> bool`; `run(args) -> int`; `main(argv=None) -> int`. Exit codes: 0 complete, 2 no simulation, 3 unresponsive. CLI: `benchmark.py run --task {all,reach,pick_place} --trials N --seed S --out DIR` and `benchmark.py compare A B`. Host: `./run-benchmark.sh` (env `TASK`, `TRIALS`, `SEED`, `CONTAINER`).

- [ ] **Step 1: Write the failing tests**

`src/lab_simulation/test/test_benchmark.py`:

```python
"""Tests for the benchmark runner's simulation-independent parts."""

from datetime import datetime
import json

from lab_simulation import benchmark
from lab_simulation.tasks import SimulationUnresponsive, TrialResult


class CountingTask:
    """Succeeds on even indices; optionally raises SimulationUnresponsive at one index."""

    def __init__(self, name, hang_at=None):
        self.name = name
        self.hang_at = hang_at
        self.seen = []

    def run(self, rng, index):
        if index == self.hang_at:
            raise SimulationUnresponsive('no joint states for 5 s')
        self.seen.append(rng.random())
        ok = index % 2 == 0
        return TrialResult(self.name, index, ok, None if ok else 'timeout', 1.0, 0.001)


def test_execute_writes_every_trial(tmp_path):
    path = tmp_path / 'trials.jsonl'
    trials, complete = benchmark.execute([CountingTask('reach')], 4, 42, lambda rng: True, path)
    assert complete
    assert [t['index'] for t in trials] == [0, 1, 2, 3]
    assert [json.loads(line)['index'] for line in path.read_text().splitlines()] == [0, 1, 2, 3]


def test_execute_is_seeded_per_task(tmp_path):
    alone = CountingTask('pick_place')
    benchmark.execute([alone], 3, 7, lambda rng: True, tmp_path / 'a.jsonl')
    together = CountingTask('pick_place')
    benchmark.execute([CountingTask('reach'), together], 3, 7, lambda rng: True,
                      tmp_path / 'b.jsonl')
    assert alone.seen == together.seen


def test_failed_reset_records_timeout(tmp_path):
    trials, complete = benchmark.execute([CountingTask('reach')], 2, 1, lambda rng: False,
                                         tmp_path / 't.jsonl')
    assert complete
    assert all(t['failure'] == 'timeout' and t['details'] == {'stage': 'reset'} for t in trials)


def test_unresponsive_simulation_stops_and_keeps_partial_results(tmp_path):
    path = tmp_path / 'trials.jsonl'
    trials, complete = benchmark.execute([CountingTask('reach', hang_at=2)], 5, 1,
                                         lambda rng: True, path)
    assert not complete
    assert len(trials) == 2
    assert len(path.read_text().splitlines()) == 2


def test_make_run_dir_never_reuses_a_directory(tmp_path):
    now = datetime(2026, 10, 5, 12, 0, 0)
    first = benchmark.make_run_dir(tmp_path / 'results', now)
    second = benchmark.make_run_dir(tmp_path / 'results', now)
    assert first.name == '20261005-120000'
    assert second.name == '20261005-120000-1'
    assert first.is_dir() and second.is_dir()


def test_write_outputs_and_compare(tmp_path):
    trials = [TrialResult('reach', i, True, None, 2.0, 0.004).to_dict() for i in range(10)]
    a = tmp_path / 'a'
    b = tmp_path / 'b'
    a.mkdir()
    b.mkdir()
    benchmark.write_outputs(a, trials, {'real_time_factor': 1.0, 'trials_per_minute': 5.0})
    faster = [dict(t, cycle_time_s=1.0) for t in trials]
    benchmark.write_outputs(b, faster, {'real_time_factor': 4.0, 'trials_per_minute': 20.0})
    assert (a / 'report.md').read_text().startswith('# Benchmark report')
    table = benchmark.compare_dirs(a, b)
    assert '| reach cycle time mean (s) | 2.000 | 1.000 |' in table
    assert '4.00×' in table


def test_main_compare(tmp_path, capsys):
    trials = [TrialResult('reach', 0, True, None, 2.0, 0.004).to_dict()]
    for name in ('a', 'b'):
        (tmp_path / name).mkdir()
        benchmark.write_outputs(tmp_path / name, trials, {})
    assert benchmark.main(['compare', str(tmp_path / 'a'), str(tmp_path / 'b')]) == 0
    assert '| Metric | a | b |' in capsys.readouterr().out
```

- [ ] **Step 2: Run to verify failure**

```bash
lab_pytest test/test_benchmark.py
```

Expected: `ImportError: cannot import name 'benchmark'`.

- [ ] **Step 3: Implement the runner and entry point**

`src/lab_simulation/lab_simulation/benchmark.py`:

```python
"""
Benchmark command line: run seeded trials, write reports, compare runs.

Usage (inside the container, with the simulation running):
  ros2 run lab_simulation benchmark.py run --task all --trials 100 --seed 42
  ros2 run lab_simulation benchmark.py compare results/A results/B

`run` is normally started by launch/benchmark.launch.py. ROS is imported only
by `run`, so the rest of this module is testable without a simulation.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
import time
from typing import Callable, List, Sequence, Tuple

from . import metrics
from .tasks import SimulationUnresponsive, TrialResult

EXIT_OK = 0
EXIT_NO_SIMULATION = 2
EXIT_UNRESPONSIVE = 3
UNTHROTTLED_RTF = 1000


def make_run_dir(base: Path, now: datetime) -> Path:
    """Create and return a new, uniquely named run directory under base."""
    stamp = now.strftime('%Y%m%d-%H%M%S')
    candidate = base / stamp
    suffix = 1
    while candidate.exists():
        candidate = base / f'{stamp}-{suffix}'
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def execute(tasks: Sequence, trials_per_task: int, seed: int,
            reset: Callable[[random.Random], bool], trials_path: Path) -> Tuple[List[dict], bool]:
    """
    Run every task trials_per_task times, appending each trial to trials_path.

    Each task gets its own random stream, derived from the seed and the task
    name, so running one task alone reproduces the same trials as running
    all. Returns the trial dicts and whether the run completed.
    """
    trials: List[dict] = []
    with open(trials_path, 'a') as out:
        for task in tasks:
            rng = random.Random(f'{seed}-{task.name}')
            for index in range(trials_per_task):
                try:
                    if reset(rng):
                        result = task.run(rng, index)
                    else:
                        result = TrialResult(task.name, index, False, 'timeout', 0.0,
                                             details={'stage': 'reset'})
                except SimulationUnresponsive as error:
                    print(f'aborting: {error}', file=sys.stderr)
                    return trials, False
                record = result.to_dict()
                trials.append(record)
                out.write(json.dumps(record) + '\n')
                out.flush()
                print(f'{task.name} {index + 1}/{trials_per_task}: '
                      f"{'ok' if record['success'] else record['failure']}", flush=True)
    return trials, True


def provenance() -> dict:
    """Return where and how this run was produced."""
    try:
        gazebo = subprocess.run(['gz', 'sim', '--versions'], capture_output=True, text=True,
                                timeout=10).stdout.strip() or 'unknown'
    except (OSError, subprocess.SubprocessError):
        gazebo = 'unknown'
    return {
        'git_commit': os.environ.get('LAB_GIT_COMMIT', 'unknown'),
        'git_dirty': os.environ.get('LAB_GIT_DIRTY', 'unknown'),
        'arch': platform.machine(),
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'gazebo': gazebo,
        'date': datetime.now(timezone.utc).isoformat(timespec='seconds'),
    }


def write_outputs(run_dir: Path, trials: List[dict], run_info: dict) -> dict:
    """Write summary.json and report.md to run_dir and return the summary."""
    summary = metrics.summarize(trials, run_info)
    (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (run_dir / 'report.md').write_text(metrics.render_report(summary))
    return summary


def compare_dirs(a: Path, b: Path) -> str:
    """Return the Markdown comparison of two run directories."""
    summary_a = json.loads((a / 'summary.json').read_text())
    summary_b = json.loads((b / 'summary.json').read_text())
    return metrics.render_comparison(metrics.compare(summary_a, summary_b), a.name, b.name)


def set_real_time_factor(value: float) -> bool:
    """Set the Gazebo real-time factor through the set_physics service."""
    command = ['gz', 'service', '-s', '/world/table_world/set_physics',
               '--reqtype', 'gz.msgs.Physics', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '5000', '--req', f'max_step_size: 0.001, real_time_factor: {value}']
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return False
    return 'true' in result.stdout


def run(args) -> int:
    """Run the benchmark against the live simulation."""
    import rclpy
    from rclpy.parameter import Parameter

    from .arm import Arm
    from .layout import sample_layout
    from .ros_util import sim_now, spin_until
    from .scene import Scene
    from .tasks import HOME, MOVE_TIMEOUT_S, PickPlaceTask, ReachTask

    rclpy.init()
    node = rclpy.create_node(
        'benchmark', parameter_overrides=[Parameter('use_sim_time', value=True)])
    arm = Arm(node)
    scene = Scene(node)
    try:
        ready = spin_until(
            node, lambda: sim_now(node) > 0 and arm.joint_positions() is not None
            and scene.has_poses(), sim_timeout_s=float('inf'), wall_timeout_s=60.0)
    except SimulationUnresponsive:
        ready = False
    if not ready:
        print('simulation not running: no clock, joint states or piece poses', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return EXIT_NO_SIMULATION

    rtf_set = set_real_time_factor(UNTHROTTLED_RTF)
    now = (lambda: sim_now(node))
    all_tasks = {'reach': ReachTask(arm, now), 'pick_place': PickPlaceTask(arm, scene, now)}
    tasks = list(all_tasks.values()) if args.task == 'all' else [all_tasks[args.task]]

    def reset(rng: random.Random) -> bool:
        scene.detach_all()
        if not arm.move_to(HOME, MOVE_TIMEOUT_S).reached:
            return False
        return scene.reset(sample_layout(rng))

    run_dir = make_run_dir(Path(args.out), datetime.now())
    wall_start, sim_start = time.monotonic(), sim_now(node)
    trials, complete = execute(tasks, args.trials, args.seed, reset, run_dir / 'trials.jsonl')
    wall, sim = time.monotonic() - wall_start, sim_now(node) - sim_start
    run_info = {
        'tasks': [task.name for task in tasks],
        'trials_per_task': args.trials,
        'seed': args.seed,
        'complete': complete,
        'unthrottled': rtf_set,
        'wall_time_s': wall,
        'sim_time_s': sim,
        'real_time_factor': sim / wall if wall > 0 else None,
        'trials_per_minute': len(trials) / (wall / 60) if wall > 0 else None,
        'provenance': provenance(),
    }
    write_outputs(run_dir, trials, run_info)
    print(f'results: {run_dir}')
    node.destroy_node()
    rclpy.try_shutdown()
    return EXIT_OK if complete else EXIT_UNRESPONSIVE


def main(argv=None) -> int:
    """Parse arguments and run a subcommand."""
    parser = argparse.ArgumentParser(prog='benchmark', description=__doc__.splitlines()[1])
    sub = parser.add_subparsers(dest='command', required=True)
    run_parser = sub.add_parser('run', help='run trials against the live simulation')
    run_parser.add_argument('--task', choices=('all', 'reach', 'pick_place'), default='all')
    run_parser.add_argument('--trials', type=int, default=100)
    run_parser.add_argument('--seed', type=int, default=42)
    run_parser.add_argument('--out', default='results')
    compare_parser = sub.add_parser('compare', help='compare two run directories')
    compare_parser.add_argument('a', type=Path)
    compare_parser.add_argument('b', type=Path)
    # Launch appends ROS arguments; ignore them.
    args, _ = parser.parse_known_args(argv)
    if args.command == 'compare':
        print(compare_dirs(args.a, args.b), end='')
        return EXIT_OK
    return run(args)
```

`src/lab_simulation/scripts/benchmark.py` (`chmod +x`):

```python
#!/usr/bin/env python3
"""Entry point for lab_simulation.benchmark (run, report and compare benchmarks)."""

import sys

from lab_simulation.benchmark import main

if __name__ == '__main__':
    sys.exit(main())
```

In `src/lab_simulation/CMakeLists.txt`, add `scripts/benchmark.py` to `install(PROGRAMS ...)`.

- [ ] **Step 4: Run the tests**

```bash
lab_pytest test/test_benchmark.py
```

Expected: 7 passed.

- [ ] **Step 5: Add the launch file**

`src/lab_simulation/launch/benchmark.launch.py`:

```python
"""
Run the benchmark against a headless simulation, then shut everything down.

Usage:
  ros2 launch lab_simulation benchmark.launch.py
  ros2 launch lab_simulation benchmark.launch.py task:=pick_place trials:=10 seed:=7
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_launch = os.path.join(
        get_package_share_directory('lab_simulation'), 'launch', 'arm_world.launch.py')

    declare_args = [
        DeclareLaunchArgument('task', default_value='all',
                              description='all, reach or pick_place'),
        DeclareLaunchArgument('trials', default_value='100', description='Trials per task'),
        DeclareLaunchArgument('seed', default_value='42', description='Random seed'),
        DeclareLaunchArgument('out', default_value='/workspace/results',
                              description='Directory for run folders'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Show RViz while benchmarking'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items())

    benchmark = Node(
        package='lab_simulation',
        executable='benchmark.py',
        name='benchmark',
        arguments=['run',
                   '--task', LaunchConfiguration('task'),
                   '--trials', LaunchConfiguration('trials'),
                   '--seed', LaunchConfiguration('seed'),
                   '--out', LaunchConfiguration('out')],
        parameters=[{'use_sim_time': True}],
        output='screen')

    stop_when_done = RegisterEventHandler(OnProcessExit(
        target_action=benchmark,
        on_exit=[EmitEvent(event=Shutdown(reason='benchmark finished'))]))

    return LaunchDescription([*declare_args, simulation, benchmark, stop_when_done])
```

- [ ] **Step 6: Host wrapper, results volume and ignore rule**

`run-benchmark.sh` (`chmod +x`):

```bash
#!/bin/bash
# Run the benchmark in the lab container. Results land in ./results/<run>/.
#
#   ./run-benchmark.sh                          # all tasks, 100 trials each, seed 42
#   TASK=pick_place TRIALS=10 ./run-benchmark.sh
#
# Stop any interactive simulation first: two simulations would share topics.
set -euo pipefail

cd "$(dirname "$0")"

CONTAINER="${CONTAINER:-ai-robotics-lab}"
TASK="${TASK:-all}"
TRIALS="${TRIALS:-100}"
SEED="${SEED:-42}"

if docker exec "${CONTAINER}" pgrep -f "gz sim" > /dev/null; then
    echo "A simulation is already running in ${CONTAINER}. Stop it first." >&2
    exit 1
fi

commit="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
if [ -z "$(git status --porcelain 2>/dev/null)" ]; then dirty=false; else dirty=true; fi

# shellcheck disable=SC2016  # expanded inside the container, not here
docker exec \
    -e LAB_GIT_COMMIT="${commit}" -e LAB_GIT_DIRTY="${dirty}" \
    -e TASK="${TASK}" -e TRIALS="${TRIALS}" -e SEED="${SEED}" \
    "${CONTAINER}" bash -c '
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install > /dev/null
source install/setup.bash
ros2 launch lab_simulation benchmark.launch.py \
    task:="${TASK}" trials:="${TRIALS}" seed:="${SEED}" out:=/workspace/results'
```

In `docker-compose.yml`, under `volumes:` after the `./src` mount, add:

```yaml
      # Benchmark results, readable on the host.
      - ./results:/workspace/results
```

In `.gitignore`, add under the colcon section:

```gitignore

# Benchmark runs (milestone baselines are copied into benchmarks/)
results/
```

- [ ] **Step 7: Lint scripts and run the full suite**

```bash
docker run --rm -v "$PWD:/mnt:ro" -w /mnt koalaman/shellcheck:stable ./*.sh scripts/*.sh
./stop-lab.sh && ./start-lab.sh    # picks up the new results volume
lab_test
```

Expected: shellcheck silent; `lab_test` 0 failures.

- [ ] **Step 8: Smoke run**

```bash
TRIALS=5 ./run-benchmark.sh
cat results/*/report.md
```

Expected: 10 progress lines (`reach 1/5: ok` …), `results: /workspace/results/<stamp>`, then a report with both tasks, `Complete: True`, and a real-time factor well above 1. If pick-and-place succeeds in 0 of 5 trials, **stop and report** the failure reasons and the gaps recorded in `trials.jsonl` to the user before Task 11; do not loosen thresholds to make it pass.

- [ ] **Step 9: Commit**

```bash
git add src/lab_simulation/lab_simulation/benchmark.py src/lab_simulation/scripts/benchmark.py src/lab_simulation/launch/benchmark.launch.py src/lab_simulation/test/test_benchmark.py src/lab_simulation/CMakeLists.txt run-benchmark.sh docker-compose.yml .gitignore
git commit -m "feat(benchmark): add seeded benchmark runner, reports and run comparison"
```

---

### Task 11: CI

**Files:**
- Modify: `.github/workflows/ci.yml`
- Create: `.github/workflows/docker.yml`

**Interfaces:**
- Consumes: `benchmark.launch.py`; `LAB_GIT_COMMIT`/`LAB_GIT_DIRTY` environment.
- Produces: a `benchmark-report` artifact on every CI run; an image build check on relevant changes.

- [ ] **Step 1: Add the simulation job**

Append to `.github/workflows/ci.yml` (under `jobs:`):

```yaml
  simulation:
    name: Benchmark smoke test (headless simulation)
    runs-on: ubuntu-24.04
    container: ros:jazzy
    timeout-minutes: 45
    defaults:
      run:
        shell: bash
    steps:
      - uses: actions/checkout@v4
        with:
          path: src/ai-robotics-lab

      - name: Install dependencies
        run: |
          apt-get update
          rosdep update --rosdistro jazzy
          rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
          apt-get install -y --no-install-recommends python3-colcon-common-extensions

      - name: Build
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build

      - name: Run benchmark (5 trials per task)
        env:
          LAB_GIT_COMMIT: ${{ github.sha }}
          LAB_GIT_DIRTY: 'false'
        run: |
          source install/setup.bash
          timeout 1800 ros2 launch lab_simulation benchmark.launch.py \
            task:=all trials:=5 seed:=42 out:="$GITHUB_WORKSPACE/results"
          python3 - <<'EOF'
          import json, pathlib, sys
          runs = sorted(pathlib.Path('results').glob('*/summary.json'))
          if not runs:
              sys.exit('no summary.json written')
          summary = json.loads(runs[-1].read_text())
          print(runs[-1].with_name('report.md').read_text())
          sys.exit(0 if summary['run']['complete'] else 'benchmark did not complete')
          EOF

      - name: Upload report
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: benchmark-report
          path: results/
          if-no-files-found: warn
```

- [ ] **Step 2: Add the image build workflow**

`.github/workflows/docker.yml`:

```yaml
name: Docker image

on:
  push:
    branches: [main]
    paths: [Dockerfile, docker-compose.yml, .dockerignore, 'scripts/**', 'src/**']
  pull_request:
    paths: [Dockerfile, docker-compose.yml, .dockerignore, 'scripts/**', 'src/**']
  workflow_dispatch:

permissions:
  contents: read

jobs:
  build:
    name: Build image (linux/amd64)
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - uses: docker/setup-buildx-action@v3
      - uses: docker/build-push-action@v6
        with:
          context: .
          platforms: linux/amd64
          push: false
          tags: ai-robotics-lab:ci
```

- [ ] **Step 3: Commit, push and watch**

```bash
git add .github/workflows/ci.yml .github/workflows/docker.yml
git commit -m "ci: run a headless benchmark smoke test and check the image build"
git push
gh run watch "$(gh run list --limit 1 --json databaseId --jq '.[0].databaseId')" --exit-status
```

Expected: all jobs green (`test`, `lint`, `simulation`; `Docker image` runs because `Dockerfile` changed in Task 2). Download the artifact with `gh run download -n benchmark-report` and confirm `report.md` shows `Complete: True`.

---

### Task 12: Baseline, speed comparison, GUI check and README

**Files:**
- Create: `benchmarks/baseline/native/summary.json`, `benchmarks/baseline/native/report.md`, `benchmarks/baseline/emulated/summary.json`, `benchmarks/baseline/emulated/report.md`, `benchmarks/baseline/comparison.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: `run-benchmark.sh`, `benchmark.py compare`, image `ai-robotics-lab:amd64-emulated` from Task 2.
- Produces: the committed baseline every later step is measured against.

- [ ] **Step 1: Emulated run (native lab stopped, for a fair speed measurement)**

```bash
./stop-lab.sh
docker run -d --name lab-emulated --platform linux/amd64 --init --shm-size 2g \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e ROS_DOMAIN_ID=43 \
  -v "$PWD/src:/workspace/src" -v "$PWD/results:/workspace/results" \
  ai-robotics-lab:amd64-emulated sleep infinity
CONTAINER=lab-emulated ./run-benchmark.sh
docker rm -f lab-emulated
```

Expected: run completes (`Complete: True`); note the run directory name.

- [ ] **Step 2: Native run**

```bash
./start-lab.sh
./run-benchmark.sh
```

Expected: run completes; note the run directory name.

- [ ] **Step 3: Collect the baseline**

```bash
NATIVE=results/<native run>     # from Step 2
EMULATED=results/<emulated run> # from Step 1
mkdir -p benchmarks/baseline/native benchmarks/baseline/emulated
cp "$NATIVE"/summary.json "$NATIVE"/report.md benchmarks/baseline/native/
cp "$EMULATED"/summary.json "$EMULATED"/report.md benchmarks/baseline/emulated/
docker exec -e A="$(basename "$EMULATED")" -e B="$(basename "$NATIVE")" ai-robotics-lab bash -c \
  'source /workspace/install/setup.bash && ros2 run lab_simulation benchmark.py compare "/workspace/results/$A" "/workspace/results/$B"' \
  > benchmarks/baseline/comparison.md
cat benchmarks/baseline/comparison.md
```

Check `summary.json` in both: `run.complete` is `true`, `run.provenance.arch` is `aarch64` (native) and `x86_64` (emulated), and `run.provenance.git_dirty` is `false`. If dirty, commit or stash local changes and rerun.

- [ ] **Step 4: Gazebo GUI check (native)**

```bash
docker exec -d ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 launch lab_simulation arm_world.launch.py gz_gui:=true rviz:=false > /tmp/gui.log 2>&1'
```

Wait 90 s, then open `http://localhost:6080`. Record whether the Gazebo window shows the table and pieces (yes/no). Stop with `./stop-lab.sh && ./start-lab.sh`.

- [ ] **Step 5: Update the README**

1. **Requirements → host table:** change Apple Silicon to `Verified. Runs natively (arm64).`; Intel/AMD hosts stay "Expected to work natively"; remove the sentence about the base image being amd64-only and the emulation notes.
2. **Troubleshooting:** replace the Gazebo GUI row with the Step 4 result (if it renders: delete the row and change the `gz_gui` launch-argument description to `Also start the Gazebo GUI`; if not: keep the row, saying it does not render with software OpenGL on this host).
3. Add a section **`## Benchmark`** after "Running the tests":

````markdown
## Benchmark

`run-benchmark.sh` runs seeded trials against a headless simulation and writes
`results/<date-time>/` containing `trials.jsonl`, `summary.json` and `report.md`.

```bash
./run-benchmark.sh                              # both tasks, 100 trials each, seed 42
TASK=pick_place TRIALS=10 ./run-benchmark.sh    # quick check
```

| Task | What happens | Success means |
|---|---|---|
| `reach` | Tip moves to a random reachable point | Tip within 1 cm |
| `pick_place` | Suction pick of a random piece, place at a random target | Grasped within 5 mm, not dropped, placed within 2 cm |

Failures are classified as `unreachable`, `grasp_miss`, `dropped`,
`placement_error` or `timeout`. Success rates carry 95% Wilson intervals.
Robot timings are in simulation seconds; trials per minute and real-time
factor are wall-clock. The benchmark lifts Gazebo's real-time cap.

Compare two runs (changes inside the confidence interval are marked
"not significant"):

```bash
docker exec ai-robotics-lab bash -c 'source /workspace/install/setup.bash && ros2 run lab_simulation benchmark.py compare /workspace/results/A /workspace/results/B'
```

The committed baseline in `benchmarks/baseline/` is the reference for all later
improvements.
````

4. **Using the lab:** add a subsection `### Watching a pick`: RViz shows the table and pieces at their live positions; to watch picks, run `./enter-lab.sh`, then `ros2 launch lab_simulation benchmark.launch.py task:=pick_place trials:=5 rviz:=true`, and open `http://localhost:6080`.
5. **Topics table:** add rows for `/lab/pieces/<piece>/pose`, `/simple_arm/suction/<piece>/{attach,detach}`, `/lab/piece_markers`, and the `set_pose` service.
6. **Status table:** add `Suction pick-and-place visible in RViz | Working` and `Seeded benchmark with committed baseline | Working (see benchmarks/baseline/)`; update the test count to the number `lab_test` reports.
7. **Project structure:** add `run-benchmark.sh`, `benchmarks/`, `lab_simulation/` (Python package), the new scripts, launch file and tests.
8. **Robot model table:** shoulder and elbow range `±3π/4 (±135°)`.

- [ ] **Step 6: Verify and commit**

```bash
lab_test
git add benchmarks README.md
git commit -m "docs: add step 1 benchmark baseline and document the benchmark

Baseline: 100 trials per task, seed 42, native arm64 and emulated amd64,
with the comparison table."
git push
```

Expected: `lab_test` 0 failures; CI green.

- [ ] **Step 7: Update local notes (not committed)**

Update `CLAUDE.md`: the test count, the native image (remove the amd64 emulation facts), the Gazebo GUI result, `run-benchmark.sh`, and that `move_arm.py`/benchmark runs must not overlap. Confirm `git status` does not list `CLAUDE.md`.
