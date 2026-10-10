# M3b Language Implementation Plan

> Implementation plan. Tasks were carried out in order, each test-first; checkboxes (`- [ ]`) track progress.

**Goal:** A typed or spoken instruction is planned by Gemma 4 E2B, checked by a deterministic safety check, and executed by the simulated SO-101, with planner and end-to-end baselines committed.

**Architecture:** Pure-Python units in `lab_simulation` (`targets`, `memory`, `plan_check`, `planner`, `executor`, `instructor`) composed by a thin ROS node serving the `/lab/instruct` action (new package `lab_interfaces`). A host voice client turns speech into action goals. Both benchmarks reuse the same `Instructor`.

**Tech Stack:** ROS 2 Jazzy (rclpy actions, rosidl), Gazebo Harmonic, MoveIt 2, Ollama 0.33 + `gemma4:e2b` on the host, mlx-whisper / faster-whisper on the host, pytest.

**Spec:** `docs/design/specs/2026-10-09-m3b-language-design.md`

## Global Constraints

- Conventional Commits, one commit per task, each builds and passes its tests; identity `Sandeep Donepudi <sandeep@strawhat.xyz>`.
- Commits carrying PR #3 code add `Co-authored-by: MunuriGangadhar <87684192+MunuriGangadhar@users.noreply.github.com>` (no personal e-mail addresses).
- Model `gemma4:e2b`; Ollama URL default `http://host.docker.internal:11434`; `num_predict` 1024; `temperature` 0; `num_ctx` 4096; thinking on (Ollama default; never send `think: false`); timeout 30 s; one retry.
- The model never supplies coordinates; `xy` targets are created only inside `plan_check` (swaps).
- In vision mode nothing the planner, check or executor uses comes from ground truth; ground truth is used only for scoring.
- Benchmark speed stays `arm.VELOCITY_SCALING` 0.5; `arm.py` is not changed.
- Dockerfile unchanged (the planner uses `urllib`); no EmbeddingGemma, `sentence-transformers` or `torchvision`.
- No `/lab/plan`, `/lab/scene` or `/lab/instruction` topics.
- `colcon test` must pass (650 checks today, more after this plan); Hadolint/ShellCheck at CI's thresholds.
- The 42 held-out planner cases (spike TEST set) are frozen: never edited, never used to tune the prompt.

## Review Focus

1. **A goal sent while another runs** must be rejected at once (outcome visible as "busy"), never queued or interleaved. Test: Task 6 `test_second_run_is_rejected_while_busy`.
2. **A remembered in-box piece is picked from its estimated pose, not seen** — if the estimate is off, the pick fails cleanly (`failed`, arm home), never a crash or a pick from ground truth. Test: Task 5 `test_box_pick_uses_remembered_pose_and_failure_homes`.
3. **Text with quotes, newlines or non-Latin script** must reach the robot unchanged through `docker exec`. Test: Task 7 `test_goal_command_passes_text_without_shell_quoting`.
4. **Ollama going away mid-benchmark** must count as a `planner_unavailable` trial failure and the run continues. Test: Task 8 `test_unavailable_planner_is_a_failed_case_not_an_abort`.
5. **A piece the camera lost track of** (memory says table, camera does not see it) must make plans that involve it refuse ("not visible"), never act on a stale position. Test: Task 3 `test_table_piece_not_seen_becomes_unknown` and Task 4 `test_unknown_piece_refuses`.

---

### Task 1: Groundwork from PR #3 (regions, targets, picking from a box)

**Files:**
- Modify: `src/lab_simulation/lab_simulation/layout.py` (add regions)
- Create: `src/lab_simulation/lab_simulation/targets.py`
- Modify: `src/lab_simulation/lab_simulation/tasks.py` (public `pick_and_place`, `from_bin`, `_approach_yaws`, placed pose)
- Modify: `src/lab_simulation/test/conftest.py` (move `FakeWorld` + `world` fixture here)
- Modify: `src/lab_simulation/test/test_tasks.py`, `src/lab_simulation/test/test_layout.py`
- Create: `src/lab_simulation/test/test_targets.py`

**Interfaces:**
- Produces: `layout.REGIONS = ('left', 'right', 'front', 'back', 'center')`, `layout.region_of(xy) -> str`, `layout.FREE_BEARINGS = (-1.73, -0.05)`.
- Produces: `targets.SIDES: Dict[str, XY]`, `targets.NEAR_GAP`, `targets.is_valid(xy, others) -> bool`, `targets.nearest_valid(desired, others) -> Optional[XY]`, `targets.region_point(region, others) -> Optional[XY]`, `targets.resolve(target: dict, piece: str, positions: Dict[str, XY], starts: Dict[str, XY]) -> Tuple[Optional[XY], str]`; target kinds `bin`, `start`, `near` (`piece`, `side` or None), `region` (`region`), `xy` (`xy`).
- Produces: `PickPlaceTask.pick_and_place(piece, start: PiecePose, target_xy, details, aim_jaws=False, nominal_grip=False, from_bin=False) -> Optional[str]` (renamed from `_execute`); on success `details['placed'] = [x, y, z, yaw]` — the piece's estimated resting pose computed from the release TCP and the grip used (nominal in vision mode).

The free table area is the annulus sector between bearings -1.73 and -0.05 rad (left of that are the bins). Regions split that sector: `left` = bearing above -0.61 (nearest the bins), `right` = below -1.17; in the middle third `front` = radius > 0.192, `back` = radius < 0.168, `center` between. "left/right/front/back" are the robot's view (front = away from it).

- [ ] **Step 1: Move the task fakes into conftest.** Cut `FakeWorld`, `HOME`, `OPEN`, `CLOSED`, `STALL`, `PRESSED` and the `world` fixture from `test/test_tasks.py` into `test/conftest.py` unchanged; in `test_tasks.py` import the constants with `from conftest import CLOSED, HOME, OPEN, PRESSED, STALL, FakeWorld`. Run `cd /workspace && colcon test --packages-select lab_simulation --pytest-args -k test_tasks && colcon test-result --verbose`; expected: all previous task tests pass.

- [ ] **Step 2: Write failing tests for regions** in `test/test_layout.py`:

```python
def test_every_region_has_free_reachable_spots():
    from lab_simulation import targets
    for region in layout.REGIONS:
        xy = targets.region_point(region, [])
        assert xy is not None, region
        assert layout.region_of(xy) == region


def test_regions_follow_the_robots_view():
    from lab_simulation import targets
    p = {r: targets.region_point(r, []) for r in layout.REGIONS}
    assert p['left'][1] > p['center'][1] > p['right'][1]
    reach = lambda xy: math.dist(xy, layout.PAN_AXIS_XY)  # noqa: E731
    assert reach(p['front']) > reach(p['center']) > reach(p['back'])


def test_no_sampled_layout_position_is_outside_every_region():
    rng = random.Random(3)
    for _ in range(200):
        for xy in layout.sample_layout(rng).values():
            assert layout.region_of(xy) in layout.REGIONS
```

- [ ] **Step 3: Implement regions** in `layout.py` (after `in_workspace`):

```python
REGIONS = ('left', 'right', 'front', 'back', 'center')
# Free table area: the annulus sector right of the bins (bearings in rad).
FREE_BEARINGS = (-1.73, -0.05)
_THIRDS = (-1.17, -0.61)
_RADIUS_BANDS = (0.168, 0.192)


def region_of(xy: Sequence[float]) -> str:
    """Return the area of the table xy lies in, as the robot sees it."""
    dx, dy = xy[0] - PAN_AXIS_XY[0], xy[1] - PAN_AXIS_XY[1]
    bearing, radius = math.atan2(dy, dx), math.hypot(dx, dy)
    if bearing > _THIRDS[1]:
        return 'left'
    if bearing < _THIRDS[0]:
        return 'right'
    if radius > _RADIUS_BANDS[1]:
        return 'front'
    return 'back' if radius < _RADIUS_BANDS[0] else 'center'
```

- [ ] **Step 4: Write `test/test_targets.py`** (PR #3's tests adapted to `layout` regions):

```python
"""Resolving plan targets to safe table positions."""

import math

from lab_simulation import layout, targets

POSITIONS = {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15),
             'green_block': (0.05, -0.19)}
STARTS = dict(POSITIONS)


def _others(piece, positions=POSITIONS):
    return [xy for name, xy in positions.items() if name != piece]


def _resolve(piece, target, positions=POSITIONS):
    return targets.resolve(target, piece, positions, STARTS)


def test_bin_target_is_the_bin_centre():
    assert _resolve('red_cube', {'kind': 'bin'})[0] == layout.BINS['red_cube'].xy


def test_start_is_used_when_free():
    assert _resolve('red_cube', {'kind': 'start'})[0] == STARTS['red_cube']


def test_start_occupied_moves_to_a_valid_nearby_point():
    positions = dict(POSITIONS, blue_cylinder=STARTS['red_cube'])
    xy, note = targets.resolve({'kind': 'start'}, 'red_cube', positions, STARTS)
    assert xy != STARTS['red_cube'] and note.startswith('moved')
    assert targets.is_valid(xy, _others('red_cube', positions))


def test_near_puts_the_piece_on_the_requested_side():
    xy, _ = _resolve('blue_cylinder', {'kind': 'near', 'piece': 'red_cube', 'side': 'back'})
    other = POSITIONS['red_cube']
    assert math.dist(xy, other) >= layout.MIN_SEPARATION
    assert math.dist(xy, layout.PAN_AXIS_XY) < math.dist(other, layout.PAN_AXIS_XY)


def test_near_without_a_side_takes_any_free_side():
    xy, _ = _resolve('green_block', {'kind': 'near', 'piece': 'red_cube', 'side': None})
    assert xy is not None and targets.is_valid(xy, _others('green_block'))


def test_near_itself_unknown_or_bad_side_is_refused():
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'red_cube', 'side': 'left'})[0] is None
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'nope', 'side': 'left'})[0] is None
    assert _resolve('red_cube', {'kind': 'near', 'piece': 'green_block',
                                 'side': 'up'})[0] is None


def test_every_region_resolves_in_that_region():
    for region in layout.REGIONS:
        xy, _ = _resolve('green_block', {'kind': 'region', 'region': region})
        assert xy is not None and layout.region_of(xy) == region
        assert targets.is_valid(xy, _others('green_block'))


def test_unreachable_xy_is_moved_into_the_workspace():
    xy, _ = _resolve('red_cube', {'kind': 'xy', 'xy': (2.0, 2.0)})
    assert targets.is_valid(xy, _others('red_cube'))


def test_xy_inside_a_bin_is_moved_clear_of_it():
    xy, _ = _resolve('red_cube', {'kind': 'xy', 'xy': layout.BINS['green_block'].xy})
    assert layout.clear_of_bins(xy)


def test_unknown_kind_is_refused():
    xy, reason = _resolve('red_cube', {'kind': 'sideways'})
    assert xy is None and reason
```

- [ ] **Step 5: Create `targets.py`** (PR #3's module; regions now from `layout`; `near` with no side picks the nearest valid spot around the other piece):

```python
"""
Resolve a plan's place target to a safe table position.

Pure Python (no ROS). A safe position is reachable with a vertical grasp,
inside the free table area, clear of the bins and of every other piece.
Targets that are not safe move to the nearest position that is.
From PR #3 by MunuriGangadhar.
"""

import math
from typing import Dict, Optional, Sequence, Tuple

from lab_simulation import layout

XY = Tuple[float, float]
EDGE_MARGIN = 0.005
NEAR_GAP = layout.MIN_SEPARATION + 0.01
GRID_STEP = 0.005
# Unit directions, world frame, as the robot sees them: front = away (+x).
SIDES: Dict[str, XY] = {'left': (0.0, 1.0), 'right': (0.0, -1.0),
                        'front': (1.0, 0.0), 'back': (-1.0, 0.0)}
_REGION_ANCHORS = {'left': (-0.33, 0.18), 'right': (-1.45, 0.18), 'front': (-0.89, 0.205),
                   'back': (-0.89, 0.155), 'center': (-0.89, 0.18)}   # (bearing, radius)


def is_valid(xy: Sequence[float], others: Sequence[Sequence[float]]) -> bool:
    """Return True if xy is reachable, in the free area and clear of bins and pieces."""
    dx, dy = xy[0] - layout.PAN_AXIS_XY[0], xy[1] - layout.PAN_AXIS_XY[1]
    bearing = math.atan2(dy, dx)
    return (layout.MIN_RADIUS + EDGE_MARGIN <= math.hypot(dx, dy)
            <= layout.MAX_RADIUS - EDGE_MARGIN
            and layout.FREE_BEARINGS[0] <= bearing <= layout.FREE_BEARINGS[1]
            and layout.clear_of_bins(xy)
            and all(math.dist(xy, other) >= layout.MIN_SEPARATION for other in others))


def _grid():
    steps = int(2 * layout.MAX_RADIUS / GRID_STEP) + 1
    x0 = layout.PAN_AXIS_XY[0] - layout.MAX_RADIUS
    y0 = layout.PAN_AXIS_XY[1] - layout.MAX_RADIUS
    for i in range(steps):
        for j in range(steps):
            yield (x0 + i * GRID_STEP, y0 + j * GRID_STEP)


def nearest_valid(desired: Sequence[float], others, region: Optional[str] = None
                  ) -> Optional[XY]:
    """Return the valid position closest to desired (inside region if given), or None."""
    best, best_distance = None, math.inf
    for xy in _grid():
        distance = math.dist(xy, desired)
        if (distance < best_distance and is_valid(xy, others)
                and (region is None or layout.region_of(xy) == region)):
            best, best_distance = xy, distance
    return best


def region_point(region: str, others) -> Optional[XY]:
    """Return the valid position nearest the middle of a region, or None."""
    bearing, radius = _REGION_ANCHORS[region]
    anchor = (layout.PAN_AXIS_XY[0] + radius * math.cos(bearing),
              layout.PAN_AXIS_XY[1] + radius * math.sin(bearing))
    return nearest_valid(anchor, others, region)


def resolve(target: dict, piece: str, positions: Dict[str, XY],
            starts: Dict[str, XY]) -> Tuple[Optional[XY], str]:
    """
    Return (xy, note) for a target of piece, or (None, reason).

    positions are the other pieces' current table positions (pieces in bins
    are absent) and starts where they began. A bin target returns that bin's
    centre unchecked (the arm aims its jaws there).
    """
    kind = target.get('kind')
    others = [xy for name, xy in positions.items() if name != piece]
    if kind == 'bin':
        return layout.BINS[piece].xy, 'bin'
    if kind == 'start':
        desired = starts.get(piece)
        if desired is None:
            return None, f'no start position known for {piece}'
    elif kind == 'near':
        other, side = target.get('piece'), target.get('side')
        if other not in positions or other == piece:
            return None, f'cannot place {piece} near {other}'
        if side is None:
            xy = nearest_valid(positions[other], others)
            return (xy, 'near') if xy else (None, f'no free spot near {other}')
        if side not in SIDES:
            return None, f'unknown side {side}'
        d = SIDES[side]
        desired = (positions[other][0] + NEAR_GAP * d[0], positions[other][1] + NEAR_GAP * d[1])
    elif kind == 'region':
        if target.get('region') not in layout.REGIONS:
            return None, f'unknown region {target.get("region")}'
        xy = region_point(target['region'], others)
        return (xy, 'region') if xy else (None, f'no free reachable spot for {piece}')
    elif kind == 'xy':
        desired = tuple(target['xy'])
    else:
        return None, f'unknown target kind {kind}'
    if is_valid(desired, others):
        return tuple(desired), 'as asked'
    xy = nearest_valid(desired, others)
    if xy is None:
        return None, f'no free reachable spot for {piece}'
    return xy, 'moved to the nearest free reachable spot'
```

- [ ] **Step 6: Run** `colcon test --packages-select lab_simulation --pytest-args -k "targets or layout"`; expected PASS (if `test_every_region_has_free_reachable_spots` fails for one region, adjust that region's anchor in `_REGION_ANCHORS`, never the thresholds' meaning).

- [ ] **Step 7: Write failing task tests** in `test/test_tasks.py`:

```python
def test_pick_and_place_records_the_placed_pose(world):
    task = pick_task(world)
    piece = layout.PIECES['red_cube']
    details = {}
    start = world.pose('red_cube')
    assert task.pick_and_place(piece, start, layout.BINS['red_cube'].xy, details,
                               aim_jaws=True, nominal_grip=True) is None
    placed = details['placed']
    assert math.dist(placed[:2], world.pose('red_cube').xy) < 0.01


def test_from_bin_tries_other_closing_directions_after_a_planning_failure(world):
    from lab_simulation.tasks import _approach_yaws
    yaws = _approach_yaws([0.3], 0.3)
    assert [round(y[0], 3) for y in yaws] == [0.3, round(0.3 + math.pi, 3)]
    assert len(_approach_yaws(None, 0.0)) == 4


def test_from_bin_opens_the_jaws_only_partly(world, monkeypatch):
    calls = []
    original = world.release
    monkeypatch.setattr(world, 'release', lambda angle=OPEN: calls.append(angle) or original(angle))
    task = pick_task(world)
    task.pick_and_place(layout.PIECES['red_cube'], world.pose('red_cube'),
                        layout.BINS['red_cube'].xy, {}, aim_jaws=True, from_bin=True)
    assert calls[0] == BIN_RELEASE_RAD
```

(Import `BIN_RELEASE_RAD` from `lab_simulation.tasks`.)

- [ ] **Step 8: Implement in `tasks.py`.** Rename `_execute` to `pick_and_place` (update `PickPlaceTask.run`, `SortTask.run`, and every test reference). Add PR #3's `_approach_yaws` above `vertical_rotation`:

```python
def _approach_yaws(yaws: Optional[List[float]], piece_yaw: float) -> List[List[float]]:
    """
    Return closing yaws to try one by one when picking out of a bin.

    Each allowed yaw (any quarter turn for a cylinder) and its half turn,
    which puts the fixed and the moving jaw on opposite sides. From PR #3.
    """
    base = yaws if yaws is not None else [piece_yaw + k * math.pi / 2 for k in range(2)]
    return [[yaw + turn] for yaw in base for turn in (0.0, math.pi)]
```

Replace the approach block of `pick_and_place` with PR #3's loop (release first — `BIN_RELEASE_RAD` when `from_bin` — then for each attempt in `_approach_yaws(yaws, start.yaw) if from_bin else [yaws]`: solve grasp and hover, `continue` on `Unreachable`, move hover then grasp, stop trying unless the failure is `'planning_failed'`; return the last failure). After a successful release-and-retreat, before homing, record:

```python
        # Where the piece now rests, estimated from the release pose and the
        # grip used (nominal in vision mode): the memory picks it from here.
        tcp_rot = tcp[:3, :3]
        centre = tcp[:3, 3] + tcp_rot @ (-grip)
        piece_yaw = start.yaw + (release_yaw - self._kin.grasp_yaw(q_grasp))
        details['placed'] = [float(centre[0]), float(centre[1]), piece.centre_z, piece_yaw]
```

(`tcp` is the release TCP computed just before the slide; `q_grasp` is the grasp configuration of the successful attempt — keep it in a variable outside the attempt loop. `grip` is the grasp-frame offset of the piece from the TCP as used by `_tcp_for_piece`; if the sign convention there is `piece = tcp + R @ grip`, use `+grip` — match `_tcp_for_piece`.)

- [ ] **Step 9: Run** `colcon test --packages-select lab_simulation && colcon test-result --verbose`; expected all PASS, including the existing sort and pick_place tests.

- [ ] **Step 10: Grasping regression (live, ≥ 20 trials).** On the host: `TASK=pick_place TRIALS=20 ./run-benchmark.sh` then `TASK=sort TRIALS=20 ./run-benchmark.sh`. Expected: pick_place ≥ 18/20 (step 2 baseline 98%), sort ≥ 17/20 (93%). If below, investigate before committing.

- [ ] **Step 11: Commit**

```bash
git add src/lab_simulation
git commit -m "feat(tasks): add safe place targets, table regions and picking from a box

Co-authored-by: MunuriGangadhar <87684192+MunuriGangadhar@users.noreply.github.com>"
```

---

### Task 2: `lab_interfaces` package with the Instruct action

**Files:**
- Create: `src/lab_interfaces/package.xml`, `src/lab_interfaces/CMakeLists.txt`, `src/lab_interfaces/action/Instruct.action`
- Modify: `src/lab_simulation/package.xml` (`<exec_depend>lab_interfaces</exec_depend>`, `<test_depend>lab_interfaces</test_depend>`)
- Create: `src/lab_simulation/test/test_interfaces.py`

**Interfaces:**
- Produces: `lab_interfaces.action.Instruct` — Goal `string text`; Result `string outcome`, `string message`, `string[] steps`; Feedback `string stage`, `string message`.

- [ ] **Step 1: Write the failing test** `test/test_interfaces.py`:

```python
"""The Instruct action interface exists with the agreed fields."""

from lab_interfaces.action import Instruct


def test_instruct_fields_round_trip():
    goal = Instruct.Goal(text='put the red cube in its box')
    result = Instruct.Result(outcome='done', message='ok', steps=['move red_cube to bin'])
    feedback = Instruct.Feedback(stage='planning', message='')
    assert goal.text.startswith('put')
    assert result.steps == ['move red_cube to bin'] and result.outcome == 'done'
    assert feedback.stage == 'planning'
```

- [ ] **Step 2: Create the package.** `action/Instruct.action`:

```
# A spoken or typed instruction for the robot.
string text
---
# done | answered | refused | not_understood | failed | planner_unavailable | cancelled
string outcome
string message
string[] steps
---
# heard | planning | plan | step | done
string stage
string message
```

`CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(lab_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "action/Instruct.action")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

`package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lab_interfaces</name>
  <version>0.3.0</version>
  <description>Interfaces of the AI and Robotics Lab: the Instruct action.</description>
  <maintainer email="sandeep@strawhat.xyz">Sandeep Donepudi</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>action_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Build and test** `colcon build --symlink-install && colcon test --packages-select lab_interfaces lab_simulation && colcon test-result --verbose`; expected PASS. Rehearse CI deps: `rosdep install --from-paths src --ignore-src -y --simulate | grep -i rosidl` shows the generators resolve.

- [ ] **Step 4: Commit** `git commit -m "feat(interfaces): add the Instruct action"`.

---

### Task 3: Placement memory

**Files:**
- Create: `src/lab_simulation/lab_simulation/memory.py`, `src/lab_simulation/test/test_memory.py`

**Interfaces:**
- Consumes: `tasks.PiecePose`.
- Produces: `memory.TABLE`, `memory.BOX`, `memory.UNKNOWN`; `PieceState(name, status, pose: Optional[PiecePose], start_xy: Optional[XY], placed_by_me: bool)`; `PlacementMemory` with `reset()`, `observe(seen: Dict[str, PiecePose])`, `placed_in_box(name, pose: PiecePose)`, `moved(name)`, `states() -> Dict[str, PieceState]` (every piece in `layout.PIECES`).

- [ ] **Step 1: Write the failing tests** `test/test_memory.py`:

```python
"""Placement memory: what the robot believes about pieces it cannot see."""

from lab_simulation.memory import BOX, PlacementMemory, TABLE, UNKNOWN
from lab_simulation.tasks import PiecePose

RED = PiecePose(0.20, -0.08, 0.765)
BOXED = PiecePose(0.135, 0.176, 0.765)


def test_everything_is_unknown_until_seen():
    memory = PlacementMemory()
    assert {s.status for s in memory.states().values()} == {UNKNOWN}


def test_first_sighting_sets_the_start():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.observe({'red_cube': PiecePose(0.15, -0.10, 0.765)})
    state = memory.states()['red_cube']
    assert state.status == TABLE and state.start_xy == RED.xy
    assert state.pose.xy == (0.15, -0.10)


def test_placed_in_box_survives_not_being_seen():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.moved('red_cube')
    memory.placed_in_box('red_cube', BOXED)
    memory.observe({})
    state = memory.states()['red_cube']
    assert state.status == BOX and state.placed_by_me and state.pose == BOXED


def test_camera_wins_over_memory():
    memory = PlacementMemory()
    memory.placed_in_box('red_cube', BOXED)
    memory.observe({'red_cube': RED})
    assert memory.states()['red_cube'].status == TABLE


def test_table_piece_not_seen_becomes_unknown():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.observe({})
    assert memory.states()['red_cube'].status == UNKNOWN


def test_moved_forgets_the_pose_until_seen_again():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.moved('red_cube')
    assert memory.states()['red_cube'].status == UNKNOWN
    assert memory.states()['red_cube'].start_xy == RED.xy


def test_reset_clears_states_and_starts():
    memory = PlacementMemory()
    memory.observe({'red_cube': RED})
    memory.reset()
    state = memory.states()['red_cube']
    assert state.status == UNKNOWN and state.start_xy is None
```

- [ ] **Step 2: Run** `colcon test --packages-select lab_simulation --pytest-args -k memory`; expected FAIL (module missing).

- [ ] **Step 3: Implement** `memory.py`:

```python
"""
Placement memory: what the robot believes about each piece.

Pure Python. The camera cannot see inside the bins, so the robot remembers
the pieces it put there. Anything the camera sees on the table wins over
memory. A piece's start is where the camera first saw it after a reset.
"""

from typing import Dict, NamedTuple, Optional, Tuple

from lab_simulation import layout
from lab_simulation.tasks import PiecePose

TABLE, BOX, UNKNOWN = 'table', 'box', 'unknown'
XY = Tuple[float, float]


class PieceState(NamedTuple):
    """One piece as the robot believes it to be."""

    name: str
    status: str                       # TABLE, BOX or UNKNOWN
    pose: Optional[PiecePose]         # seen (TABLE) or estimated at placement (BOX)
    start_xy: Optional[XY]
    placed_by_me: bool = False


class PlacementMemory:
    """Combine camera sightings with the robot's own placements."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        """Forget everything (a new scene)."""
        self._status = {name: UNKNOWN for name in layout.PIECES}
        self._pose: Dict[str, Optional[PiecePose]] = {name: None for name in layout.PIECES}
        self._start: Dict[str, Optional[XY]] = {name: None for name in layout.PIECES}
        self._placed = {name: False for name in layout.PIECES}

    def observe(self, seen: Dict[str, PiecePose]) -> None:
        """Apply one camera observation (pieces seen on the table)."""
        for name in layout.PIECES:
            if name in seen:
                self._status[name], self._pose[name] = TABLE, seen[name]
                self._placed[name] = False
                if self._start[name] is None:
                    self._start[name] = seen[name].xy
            elif self._status[name] == TABLE:
                self._status[name], self._pose[name] = UNKNOWN, None

    def placed_in_box(self, name: str, pose: PiecePose) -> None:
        """Record that the robot put name into its bin, resting at pose."""
        self._status[name], self._pose[name], self._placed[name] = BOX, pose, True

    def moved(self, name: str) -> None:
        """Forget where name is: it was picked up (the camera will find it)."""
        self._status[name], self._pose[name], self._placed[name] = UNKNOWN, None, False

    def states(self) -> Dict[str, PieceState]:
        """Return every piece's current state."""
        return {name: PieceState(name, self._status[name], self._pose[name],
                                 self._start[name], self._placed[name])
                for name in layout.PIECES}
```

- [ ] **Step 4: Run** the memory tests; expected PASS. **Step 5: Commit** `git commit -m "feat(instruct): remember pieces the robot put in their boxes"`.

---

### Task 4: Safety check

**Files:**
- Create: `src/lab_simulation/lab_simulation/plan_check.py`, `src/lab_simulation/test/test_plan_check.py`

**Interfaces:**
- Consumes: `memory.PieceState`, `memory.TABLE/BOX`, `targets.resolve`, `targets.region_point`, `targets.SIDES`, `layout.REGIONS`, `layout.PIECES`.
- Produces: `ToolCall(name: str, args: dict)` (defined here; the planner imports it); `Step(action: str, piece: Optional[str] = None, target: Optional[dict] = None)` with actions `'move'`/`'home'`; `Verdict(kind: str, steps: Tuple[Step, ...] = (), message: str = '', source: str = '', final: Dict[str, object] = {})` with kind `'plan'|'refuse'|'answer'`, source `'model'|'check'|''`, and `final` = dry-run end state per piece (`'box'`, an `(x, y)` or `None` for unknown); `check(calls: Sequence, states: Dict[str, PieceState]) -> Verdict`; `describe_step(step) -> str`; `MAX_STEPS = 8`.

- [ ] **Step 1: Write the failing tests** `test/test_plan_check.py`:

```python
"""The safety check: every malformed, unknown or impossible plan is refused."""

import random

from lab_simulation import layout
from lab_simulation.memory import BOX, PieceState, TABLE, UNKNOWN
from lab_simulation.plan_check import MAX_STEPS, check, Step, ToolCall
from lab_simulation.tasks import PiecePose

XY = {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15), 'green_block': (0.05, -0.19)}


def states(**override):
    out = {n: PieceState(n, TABLE, PiecePose(*xy, 0.765), xy) for n, xy in XY.items()}
    for name, status in override.items():
        pose = PiecePose(*layout.BINS[name].xy, 0.765) if status == BOX else None
        out[name] = PieceState(name, status, pose, XY[name], status == BOX)
    return out


def call(name, **args):
    return ToolCall(name, args)


def test_move_to_bin_is_one_step():
    v = check([call('move_piece', piece='red_cube', target='bin')], states())
    assert v.kind == 'plan' and v.steps == (Step('move', 'red_cube', {'kind': 'bin'}),)
    assert v.final['red_cube'] == 'box'


def test_colour_word_names_a_piece_and_ambiguity_refuses():
    assert check([call('move_piece', piece='the red one', target='bin')],
                 states()).steps[0].piece == 'red_cube'
    assert check([call('move_piece', piece='Red Cube', target='bin')], states()).kind == 'plan'
    v = check([call('move_piece', piece='piece', target='bin')], states())
    assert v.kind == 'refuse' and v.source == 'check'


def test_refuse_and_answer_pass_through_alone():
    assert check([call('refuse', reason='unsafe')], states()).kind == 'refuse'
    v = check([call('answer', text='Two pieces.')], states())
    assert v.kind == 'answer' and v.message == 'Two pieces.'


def test_refuse_mixed_with_moves_is_refused_as_contradictory():
    v = check([call('refuse', reason='x'), call('go_home')], states())
    assert v.kind == 'refuse' and 'contradict' in v.message


def test_unknown_piece_refuses():
    v = check([call('move_piece', piece='green_block', target='bin')],
              states(green_block=UNKNOWN))
    assert v.kind == 'refuse' and 'not visible' in v.message


def test_near_needs_the_other_piece_on_the_table():
    v = check([call('move_piece', piece='red_cube', target='near', other_piece='blue_cylinder',
                    side='left')], states(blue_cylinder=BOX))
    assert v.kind == 'refuse'


def test_swap_expands_to_three_moves_and_ends_swapped():
    v = check([call('swap_pieces', piece_a='red_cube', piece_b='green_block')], states())
    assert v.kind == 'plan' and len(v.steps) == 3
    assert v.final['red_cube'] == XY['green_block'] and v.final['green_block'] == XY['red_cube']


def test_swap_needs_both_on_the_table():
    v = check([call('swap_pieces', piece_a='red_cube', piece_b='green_block')],
              states(red_cube=BOX))
    assert v.kind == 'refuse'


def test_sort_all_moves_only_table_pieces():
    v = check([call('sort_all')], states(red_cube=BOX))
    assert [s.piece for s in v.steps] == ['blue_cylinder', 'green_block']


def test_moving_a_boxed_piece_to_its_box_is_a_no_op():
    v = check([call('move_piece', piece='red_cube', target='bin')], states(red_cube=BOX))
    assert v.kind == 'plan' and v.steps == ()


def test_too_many_steps_refuses():
    calls = [call('go_home')] * (MAX_STEPS + 1)
    assert check(calls, states()).kind == 'refuse'


def test_no_calls_is_not_a_plan():
    assert check([], states()).kind == 'refuse'


def test_random_junk_never_raises():
    rng = random.Random(7)
    junk = [None, 3, 2.5, [], {}, {'a': [1]}, 'x' * 10000, ['red_cube'], {'kind': 'bin'},
            'red_cube', 'bin', 'near', 'left', '', '\x00', '🙂']
    names = ['move_piece', 'swap_pieces', 'sort_all', 'go_home', 'refuse', 'answer', 'rm', None,
             3, []]
    keys = ['piece', 'target', 'other_piece', 'side', 'region', 'piece_a', 'piece_b', 'reason',
            'text', 'xy', 'kind']
    for _ in range(5000):
        calls = []
        for _ in range(rng.randint(0, 4)):
            args = {rng.choice(keys): rng.choice(junk) for _ in range(rng.randint(0, 4))}
            calls.append(ToolCall(rng.choice(names), rng.choice([args, args, None, 'x', 5])))
        verdict = check(calls, states())
        assert verdict.kind in ('plan', 'refuse', 'answer')
        for step in verdict.steps:
            assert step.action in ('move', 'home')
```

- [ ] **Step 2: Run**; expected FAIL (module missing).

- [ ] **Step 3: Implement** `plan_check.py`:

```python
"""
The safety check between the language model and the arm.

Pure Python. Turns the model's tool calls into validated steps, or refuses
the whole plan with a reason. It never raises on bad input: whatever the
model returns, the answer is a plan, a refusal or an answer. The model never
supplies coordinates; xy targets are created here only for swaps.
"""

from typing import Dict, NamedTuple, Optional, Sequence, Tuple

from lab_simulation import layout, targets
from lab_simulation.memory import BOX, PieceState, TABLE

MAX_STEPS = 8
TOOLS = ('move_piece', 'swap_pieces', 'sort_all', 'go_home', 'refuse', 'answer')
MOVE_TARGETS = ('bin', 'start', 'near', 'region')
_WORDS = {name: set(name.split('_')) for name in layout.PIECES}


class ToolCall(NamedTuple):
    """One tool call from the model, as parsed."""

    name: object
    args: object


class Step(NamedTuple):
    """One validated step: move a piece to a target, or go home."""

    action: str
    piece: Optional[str] = None
    target: Optional[dict] = None


class Verdict(NamedTuple):
    """The check's decision on a reply."""

    kind: str                                   # 'plan', 'refuse' or 'answer'
    steps: Tuple[Step, ...] = ()
    message: str = ''
    source: str = ''                            # who refused: 'model' or 'check'
    final: Dict[str, object] = {}               # dry-run end state: 'box', (x, y) or None


class _Refusal(Exception):
    pass


def _text(args: dict, key: str, required: bool = True) -> Optional[str]:
    value = args.get(key)
    if value is None or (isinstance(value, str) and value.strip().lower() in ('', 'none')):
        if required:
            raise _Refusal(f'missing {key}')
        return None
    if not isinstance(value, str) or len(value) > 200:
        raise _Refusal(f'{key} must be a short text')
    return value.strip()


def _piece(word: str) -> str:
    normal = word.lower().replace(' ', '_')
    if normal in layout.PIECES:
        return normal
    tokens = set(word.lower().replace('_', ' ').split())
    hits = [name for name, words in _WORDS.items() if tokens & words]
    if len(hits) != 1:
        raise _Refusal(f'which piece is "{word}"?')
    return hits[0]


def describe_step(step: Step) -> str:
    """Return a short description of a step."""
    if step.action == 'home':
        return 'go home'
    t = step.target
    where = {'bin': 'its box', 'start': 'where it started'}.get(t['kind'])
    if t['kind'] == 'near':
        where = f"{t['side'] + ' of ' if t.get('side') else 'next to '}{t['piece']}"
    elif t['kind'] == 'region':
        where = f"the {t['region']} of the table"
    elif t['kind'] == 'xy':
        where = 'a free spot' if t.get('temporary') else 'where the other piece was'
    return f"move {step.piece} to {where}"


def _steps_from(call: ToolCall, states: Dict[str, PieceState]):
    if not isinstance(call.args, dict):
        raise _Refusal('arguments must be an object')
    a = call.args
    if call.name == 'go_home':
        return [Step('home')]
    if call.name == 'sort_all':
        return [Step('move', n, {'kind': 'bin'}) for n in sorted(states)
                if states[n].status == TABLE]
    if call.name == 'swap_pieces':
        return [('swap', _piece(_text(a, 'piece_a')), _piece(_text(a, 'piece_b')))]
    piece = _piece(_text(a, 'piece'))
    kind = (_text(a, 'target') or '').lower()
    if kind not in MOVE_TARGETS:
        raise _Refusal(f'unknown target {kind}')
    if kind == 'near':
        side = _text(a, 'side', required=False)
        if side is not None and side.lower() not in targets.SIDES:
            raise _Refusal(f'unknown side {side}')
        return [Step('move', piece, {'kind': 'near', 'piece': _piece(_text(a, 'other_piece')),
                                     'side': side.lower() if side else None})]
    if kind == 'region':
        region = (_text(a, 'region') or '').lower()
        if region not in layout.REGIONS:
            raise _Refusal(f'unknown area {region}')
        return [Step('move', piece, {'kind': 'region', 'region': region})]
    return [Step('move', piece, {'kind': kind})]


def check(calls: Sequence, states: Dict[str, PieceState]) -> Verdict:
    """Return the verdict on the model's tool calls in the given scene."""
    try:
        return _check(list(calls), states)
    except _Refusal as refusal:
        return Verdict('refuse', message=str(refusal), source='check')


def _check(calls, states) -> Verdict:
    if not calls:
        raise _Refusal('no plan')
    if not all(isinstance(c, ToolCall) and isinstance(c.name, str) and c.name in TOOLS
               for c in calls):
        raise _Refusal('unknown tool')
    names = [c.name for c in calls]
    for special in ('refuse', 'answer'):
        if special in names:
            if len(calls) > 1:
                raise _Refusal('the reply contradicts itself')
            args = calls[0].args if isinstance(calls[0].args, dict) else {}
            key = 'reason' if special == 'refuse' else 'text'
            text = args.get(key) if isinstance(args.get(key), str) else ''
            return Verdict(special, message=text[:500], source='model' if special == 'refuse'
                           else '')
    raw = []
    for call in calls:
        raw.extend(_steps_from(call, states))
    return _dry_run(raw, states)


def _dry_run(raw, states) -> Verdict:
    positions = {n: s.pose.xy for n, s in states.items() if s.status == TABLE}
    location: Dict[str, object] = {n: ('box' if s.status == BOX else positions.get(n))
                                   for n, s in states.items()}
    starts = {n: s.start_xy for n, s in states.items() if s.start_xy is not None}
    steps = []

    def move(piece, target, index):
        if location[piece] is None:
            raise _Refusal(f'step {index}: {piece} is not visible')
        if target['kind'] == 'bin' and location[piece] == 'box':
            return
        xy, note = targets.resolve(target, piece, positions, starts)
        if xy is None:
            raise _Refusal(f'step {index}: {note}')
        steps.append(Step('move', piece, target))
        if target['kind'] == 'bin':
            positions.pop(piece, None)
            location[piece] = 'box'
        else:
            positions[piece] = xy
            location[piece] = xy

    for index, item in enumerate(raw, 1):
        if isinstance(item, Step) and item.action == 'home':
            steps.append(item)
        elif isinstance(item, Step):
            move(item.piece, item.target, index)
        else:
            _, a, b = item
            if a == b or location[a] in (None, 'box') or location[b] in (None, 'box'):
                raise _Refusal(f'step {index}: can only swap two pieces on the table')
            xa, xb = location[a], location[b]
            free = None
            for region in ('center', 'front', 'back', 'right', 'left'):
                spot = targets.region_point(region, [p for n, p in positions.items() if n != a]
                                            + [xa, xb])
                if spot is not None:
                    free = spot
                    break
            if free is None:
                raise _Refusal(f'step {index}: no free spot to swap')
            move(a, {'kind': 'xy', 'xy': free, 'temporary': True}, index)
            move(b, {'kind': 'xy', 'xy': xa}, index)
            move(a, {'kind': 'xy', 'xy': xb}, index)
        if len(steps) > MAX_STEPS:
            raise _Refusal(f'more than {MAX_STEPS} steps')
    return Verdict('plan', tuple(steps), final=dict(location))
```

Note: `positions` passed to `resolve` includes the moving piece itself; `resolve` excludes it from `others`. For `near`, the other piece must be in `positions` (on the table), which gives the refusal in `test_near_needs_the_other_piece_on_the_table`.

- [ ] **Step 4: Run** `colcon test --packages-select lab_simulation --pytest-args -k plan_check && colcon test-result --verbose`; expected PASS (flake8/pep257 too). **Step 5: Commit** `git commit -m "feat(instruct): check every plan before the arm moves"`.

---

### Task 5: Planner and executor

**Files:**
- Create: `src/lab_simulation/lab_simulation/planner.py`, `src/lab_simulation/lab_simulation/executor.py`
- Create: `src/lab_simulation/test/test_planner.py`, `src/lab_simulation/test/test_executor.py`

**Interfaces:**
- Consumes: `plan_check.ToolCall/Step/describe_step`, `memory.*`, `targets.resolve`, `tasks.PickPlaceTask.pick_and_place`, `tasks.BIN_RELEASE_RAD`, `layout`.
- Produces (planner): `DEFAULT_URL`, `DEFAULT_MODEL`, `NUM_PREDICT = 1024`, `TIMEOUT_S = 30.0`; `PlannerReply(calls: List[ToolCall], error: Optional[str], raw: dict, seconds: float)` with error `None|'unavailable'|'no_call'`; `describe_scene(states) -> str`; `parse_calls(message: dict) -> List[ToolCall]`; `Planner(url=DEFAULT_URL, model=DEFAULT_MODEL, timeout_s=TIMEOUT_S)` with `.check_ready() -> Optional[str]` (None when ready, else a reason) and `.plan(text, states) -> PlannerReply`.
- Produces (executor): `Executor(pick, scene, gripper, arm, memory)` with `.observe() -> Tuple[Dict[str, PieceState], Observation]`, `.run(steps, feedback: Callable[[str, str], None], cancelled: Callable[[], bool]) -> Tuple[str, str]` returning `('done'|'failed'|'cancelled', message)`, `.reset_arm() -> bool`.

- [ ] **Step 1: Write failing planner tests** `test/test_planner.py` with a fake Ollama server:

```python
"""The planner's Ollama client, against a scripted fake server."""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import threading

from lab_simulation.memory import BOX, PieceState, TABLE, UNKNOWN
from lab_simulation.planner import describe_scene, NUM_PREDICT, parse_calls, Planner
from lab_simulation.tasks import PiecePose
import pytest

STATES = {
    'red_cube': PieceState('red_cube', BOX, PiecePose(0.135, 0.176, 0.765), (0.2, -0.08), True),
    'blue_cylinder': PieceState('blue_cylinder', TABLE, PiecePose(0.12, -0.15, 0.765),
                                (0.2, -0.10)),
    'green_block': PieceState('green_block', UNKNOWN, None, None),
}


class Fake:
    """Replies to /api/chat from a script; records requests."""

    def __init__(self, replies):
        self.replies, self.requests = list(replies), []
        fake = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                body = json.dumps({'models': [{'name': 'gemma4:e2b'}]}).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                fake.requests.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
                reply = fake.replies.pop(0)
                if reply == 'hang':
                    threading.Event().wait(2.0)
                    return
                body = json.dumps({'message': reply}).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(body)

        self.server = HTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'


def tool(name, **args):
    return {'function': {'name': name, 'arguments': args}}


def test_scene_text_comes_from_memory_and_camera_only():
    text = describe_scene(STATES)
    assert 'red_cube (red cube): in its box (placed by me)' in text
    assert 'green_block (green block): not visible' in text
    assert 'blue_cylinder (blue cylinder): on the table' in text and 'moved from its start' in text


def test_native_tool_calls_are_parsed_and_limits_sent():
    fake = Fake([{'tool_calls': [tool('move_piece', piece='blue_cylinder', target='bin')]}])
    reply = Planner(fake.url).plan('put the blue one away', STATES)
    assert reply.error is None and reply.calls[0].name == 'move_piece'
    sent = fake.requests[0]
    assert sent['options']['num_predict'] == NUM_PREDICT and sent['options']['temperature'] == 0
    assert 'think' not in sent
    assert {t['function']['name'] for t in sent['tools']} >= {'refuse', 'answer'}


def test_text_written_calls_are_parsed():
    calls = parse_calls({'content': 'refuse{reason:<|"|>unsafe<|"|>} go_home{}'})
    assert [c.name for c in calls] == ['refuse', 'go_home']
    assert calls[0].args == {'reason': 'unsafe'}


def test_no_call_is_retried_once_then_not_understood():
    fake = Fake([{'content': 'hmm'}, {'content': 'still no'}])
    reply = Planner(fake.url).plan('blah', STATES)
    assert reply.error == 'no_call' and len(fake.requests) == 2


def test_timeout_is_retried_then_not_understood():
    fake = Fake(['hang', {'tool_calls': [tool('go_home')]}])
    reply = Planner(fake.url, timeout_s=0.5).plan('go home', STATES)
    assert reply.error is None and reply.calls[0].name == 'go_home'


def test_server_down_is_unavailable():
    reply = Planner('http://127.0.0.1:9', timeout_s=0.5).plan('go home', STATES)
    assert reply.error == 'unavailable'


def test_check_ready_reports_a_missing_model():
    fake = Fake([])
    assert Planner(fake.url).check_ready() is None
    assert 'pull' in Planner(fake.url, model='other:1b').check_ready()
```

- [ ] **Step 2: Implement** `planner.py`. Copy PR #3's `TOOLS` (`move_piece`, `swap_pieces`, `go_home`, `sort_all` with the same JSON schemas, `region` enum = `layout.REGIONS`) and append the spike's `refuse(reason)` and `answer(text)` tools. `SYSTEM` = PR #3's system prompt with the refusal rule replaced exactly as in the spike's `pr_plus.py` (`NEW_RULES` and the `refuse/answer` tool line) and `{scene}` filled by `describe_scene`. Core:

```python
"""
The language-model planner: an instruction and the scene in, tool calls out.

Talks to Ollama (Gemma 4 E2B) over HTTP with the standard library only.
Gemma 4 thinks by default; replies are capped (num_predict), timed out and
retried once, so a runaway reply cannot stall the robot. Prompt and tools
from PR #3 by MunuriGangadhar, plus explicit refuse and answer tools.
"""

import json
import re
import time
from typing import Dict, List, NamedTuple, Optional
import urllib.error
import urllib.request

from lab_simulation import layout
from lab_simulation.memory import BOX, PieceState, TABLE
from lab_simulation.plan_check import ToolCall

DEFAULT_URL = 'http://host.docker.internal:11434'
DEFAULT_MODEL = 'gemma4:e2b'
NUM_PREDICT = 1024
TIMEOUT_S = 30.0
ATTEMPTS = 2           # one retry
TOOL_NAMES = ('move_piece', 'swap_pieces', 'sort_all', 'go_home', 'refuse', 'answer')
_TEXT_CALL = re.compile(r'(' + '|'.join(TOOL_NAMES) + r')\s*[{(]([^})]*)[})]')
_TEXT_ARG = re.compile(r'(\w+)\s*:\s*<\|"\|>(.*?)<\|"\|>')


class PlannerReply(NamedTuple):
    """The model's tool calls, or why there are none."""

    calls: List[ToolCall]
    error: Optional[str]          # None, 'unavailable' or 'no_call'
    raw: dict
    seconds: float


def describe_scene(states: Dict[str, PieceState]) -> str:
    """Describe the pieces from camera sightings and memory only."""
    lines = []
    for name in sorted(states):
        s = states[name]
        label = f'{name} ({name.replace("_", " ")})'
        if s.status == BOX:
            where = 'in its box' + (' (placed by me)' if s.placed_by_me else '')
        elif s.status == TABLE:
            where = f'on the table, {layout.region_of(s.pose.xy)} area'
            if s.start_xy is not None and _dist(s.pose.xy, s.start_xy) > 0.03:
                where += f'; moved from its start ({layout.region_of(s.start_xy)} area)'
        else:
            where = 'not visible'
        lines.append(f'- {label}: {where}')
    return '\n'.join(lines)


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def parse_calls(message: dict) -> List[ToolCall]:
    """Return the tool calls in a reply: native, or written as text."""
    calls = []
    for call in message.get('tool_calls') or []:
        fn = call.get('function', {}) if isinstance(call, dict) else {}
        args = fn.get('arguments')
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except ValueError:
                pass
        calls.append(ToolCall(fn.get('name'), args if args is not None else {}))
    if calls:
        return calls
    for name, body in _TEXT_CALL.findall(message.get('content') or ''):
        calls.append(ToolCall(name, dict(_TEXT_ARG.findall(body))))
    return calls


class Planner:
    """Plans instructions with a local Ollama model."""

    def __init__(self, url: str = DEFAULT_URL, model: str = DEFAULT_MODEL,
                 timeout_s: float = TIMEOUT_S):
        self.url, self.model, self.timeout_s = url.rstrip('/'), model, timeout_s

    def check_ready(self) -> Optional[str]:
        """Return None if Ollama answers and has the model, else what to do."""
        try:
            with urllib.request.urlopen(self.url + '/api/tags', timeout=5) as r:
                names = {m.get('name') for m in json.load(r).get('models', [])}
        except (OSError, ValueError) as error:
            return f'Ollama not reachable at {self.url} ({error}); start Ollama'
        if self.model not in names:
            return f'model {self.model} missing; run: ollama pull {self.model}'
        return None

    def plan(self, text: str, states: Dict[str, PieceState]) -> PlannerReply:
        """Ask the model for tool calls; retry once on a timeout or no call."""
        body = {'model': self.model, 'stream': False, 'keep_alive': '30m', 'tools': TOOLS,
                'messages': [{'role': 'system',
                              'content': SYSTEM.format(scene=describe_scene(states))},
                             {'role': 'user', 'content': text}],
                'options': {'temperature': 0, 'num_ctx': 4096, 'num_predict': NUM_PREDICT}}
        t0, raw = time.monotonic(), {}
        for _ in range(ATTEMPTS):
            try:
                request = urllib.request.Request(self.url + '/api/chat', json.dumps(body).encode(),
                                                 {'Content-Type': 'application/json'})
                with urllib.request.urlopen(request, timeout=self.timeout_s) as r:
                    raw = json.load(r).get('message', {})
            except urllib.error.URLError as error:
                if not isinstance(getattr(error, 'reason', None), TimeoutError):
                    return PlannerReply([], 'unavailable', {'error': str(error)},
                                        time.monotonic() - t0)
                continue
            except (TimeoutError, ValueError):
                continue
            calls = parse_calls(raw)
            if calls:
                return PlannerReply(calls, None, raw, time.monotonic() - t0)
        return PlannerReply([], 'no_call', raw, time.monotonic() - t0)
```

(`ConnectionRefusedError` arrives as `URLError` with a non-timeout reason → `unavailable`; a socket timeout arrives as `TimeoutError` or `URLError(reason=TimeoutError)` → retried.)

- [ ] **Step 3: Run planner tests**; expected PASS.

- [ ] **Step 4: Write failing executor tests** `test/test_executor.py` using `FakeWorld` from conftest (it implements arm, gripper and scene). Add to `FakeWorld` (in conftest) the methods the executor needs if missing: `unlatch_all()`, `update_planning_scene(exclude=None, held=None, bins=True)`.

```python
"""The executor runs checked steps on the (fake) robot and keeps memory honest."""

from conftest import FakeWorld
from lab_simulation import layout
from lab_simulation.executor import Executor
from lab_simulation.memory import BOX, PlacementMemory, TABLE
from lab_simulation.plan_check import Step
from lab_simulation.tasks import PickPlaceTask
import pytest


@pytest.fixture
def setup(lab_kinematics):
    world = FakeWorld(lab_kinematics)
    pick = PickPlaceTask(world, world, world, lab_kinematics, world.now, 'latched')
    return world, Executor(pick, world, world, world, PlacementMemory())


def run(executor, steps, cancel_after=None):
    seen = []
    done = executor.run(steps, lambda stage, msg: seen.append((stage, msg)),
                        lambda: cancel_after is not None and len(seen) > cancel_after)
    return done, seen


def test_move_to_bin_updates_memory(setup):
    world, ex = setup
    ex.observe()
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'bin'})])
    assert outcome[0] == 'done'
    assert ex.memory.states()['red_cube'].status == BOX
    assert layout.BINS['red_cube'].contains(world.pose('red_cube').xy)


def test_box_pick_uses_remembered_pose_and_failure_homes(setup, monkeypatch):
    world, ex = setup
    ex.observe()
    run(ex, [Step('move', 'red_cube', {'kind': 'bin'})])
    world.fail_plan_at = len(world.moves)          # the next move fails to plan
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'start'})])
    assert outcome[0] == 'failed' and 'step 1/1' in outcome[1]
    assert world.q == world.home_q


def test_steps_report_progress_in_order(setup):
    _, ex = setup
    ex.observe()
    _, seen = run(ex, [Step('move', 'red_cube', {'kind': 'bin'}), Step('home')])
    assert [s for s, _ in seen] == ['step', 'step']
    assert seen[0][1].startswith('step 1/2')


def test_cancel_takes_effect_between_steps(setup):
    _, ex = setup
    ex.observe()
    outcome, _ = run(ex, [Step('move', 'red_cube', {'kind': 'bin'}),
                          Step('move', 'blue_cylinder', {'kind': 'bin'})], cancel_after=1)
    assert outcome[0] == 'cancelled'
    assert ex.memory.states()['red_cube'].status == BOX
    assert ex.memory.states()['blue_cylinder'].status == TABLE


def test_step_no_longer_valid_stops_the_plan(setup):
    world, ex = setup
    ex.observe()
    world.hide = {'green_block'}                    # the camera loses it
    outcome, _ = run(ex, [Step('move', 'green_block', {'kind': 'bin'})])
    assert outcome[0] == 'failed' and 'not visible' in outcome[1]
```

(Add `self.hide = set()` and `self.home_q = HOME` to `FakeWorld`; its `observe()` omits names in `hide`.)

- [ ] **Step 5: Implement** `executor.py`:

```python
"""
Run checked plan steps on the robot, one at a time.

Each step looks at the table again first; if the step is no longer valid
the plan stops and the arm goes home. Cancelling takes effect between
steps. The placement memory is updated from what actually happened.
"""

from typing import Callable, Dict, Sequence, Tuple

from lab_simulation import layout, targets
from lab_simulation.memory import BOX, PieceState, PlacementMemory, TABLE
from lab_simulation.plan_check import describe_step, Step
from lab_simulation.tasks import BIN_RELEASE_RAD, Observation, PiecePose

Feedback = Callable[[str, str], None]


class Executor:
    """Executes steps with the pick-and-place skill and keeps the memory."""

    def __init__(self, pick, scene, gripper, arm, memory: PlacementMemory):
        self._pick, self._scene, self._gripper, self._arm = pick, scene, gripper, arm
        self.memory = memory

    def observe(self) -> Tuple[Dict[str, PieceState], Observation]:
        """Look at the table; return the updated states and the observation."""
        observation = self._scene.observe()
        self.memory.observe(observation.poses)
        return self.memory.states(), observation

    def reset_arm(self) -> bool:
        """Let go, open partly and go home ignoring pieces and bins."""
        self._scene.unlatch_all()
        self._gripper.release(BIN_RELEASE_RAD)
        self._scene.update_planning_scene(exclude=list(layout.PIECES), bins=False)
        result = self._arm.home()
        self._scene.update_planning_scene()
        return result.planned and result.reached

    def run(self, steps: Sequence[Step], feedback: Feedback,
            cancelled: Callable[[], bool]) -> Tuple[str, str]:
        """Run steps in order; return (outcome, message)."""
        total = len(steps)
        for index, step in enumerate(steps, 1):
            if cancelled():
                self.reset_arm()
                return 'cancelled', f'Cancelled after step {index - 1}/{total}.'
            label = f'step {index}/{total}'
            feedback('step', f'{label}: {describe_step(step)}')
            failure = self._run_step(step)
            if failure:
                self.reset_arm()
                return 'failed', f'Failed at {label}: {failure}.'
        return 'done', 'Done: ' + '; '.join(describe_step(s) for s in steps) + '.'

    def _run_step(self, step: Step) -> str:
        if step.action == 'home':
            return '' if self.reset_arm() else 'could not go home'
        states, observation = self.observe()
        state = states[step.piece]
        if state.status not in (TABLE, BOX):
            return f'{step.piece} is not visible'
        positions = {n: s.pose.xy for n, s in states.items() if s.status == TABLE}
        starts = {n: s.start_xy for n, s in states.items() if s.start_xy is not None}
        xy, note = targets.resolve(step.target, step.piece, positions, starts)
        if xy is None:
            return note
        details: dict = {}
        to_bin = step.target['kind'] == 'bin'
        try:
            failure = self._pick.pick_and_place(
                layout.PIECES[step.piece], state.pose, xy, details, aim_jaws=to_bin,
                nominal_grip=observation.vision, from_bin=state.status == BOX)
        finally:
            self._scene.unlatch(step.piece)
            self._gripper.release(BIN_RELEASE_RAD)
            self._scene.update_planning_scene()
        if failure is None and to_bin and 'placed' in details:
            self.memory.placed_in_box(step.piece, PiecePose(*details['placed']))
        elif details.get('stage') != 'approach' or failure is None:
            self.memory.moved(step.piece)          # picked up: the camera will find it
        return f'{failure} (stage {details.get("stage")})' if failure else ''
```

- [ ] **Step 6: Run** `colcon test --packages-select lab_simulation && colcon test-result --verbose`; expected PASS. **Step 7: Commit** `git commit -m "feat(instruct): plan with Gemma 4 E2B and execute checked steps"` with the PR #3 `Co-authored-by` trailer (prompt and tools are theirs).

---

### Task 6: Instructor, instruction node and demo launch

**Files:**
- Create: `src/lab_simulation/lab_simulation/instructor.py`, `src/lab_simulation/test/test_instructor.py`
- Create: `src/lab_simulation/scripts/instruction_node.py`, `src/lab_simulation/launch/demo.launch.py`
- Modify: `src/lab_simulation/CMakeLists.txt` (install `scripts/instruction_node.py`), `docker-compose.yml` (`extra_hosts: ["host.docker.internal:host-gateway"]`, for Linux hosts)

**Interfaces:**
- Consumes: `Planner`, `check`, `Executor`, `Verdict`, `describe_step`.
- Produces: `InstructResult(outcome, message, steps: List[str], record: dict)`; `Instructor(planner, executor, log_path: Optional[Path])` with `.try_begin() -> bool`, `.end()`, `.run(text, feedback, cancelled) -> InstructResult`; `OUTCOMES` tuple. Node `instruction_node.py` serves `/lab/instruct` (`lab_interfaces/action/Instruct`), parameters `perception` (`ground_truth|vision`, default `vision`), `ollama_url`, `model`, `log` (default `/workspace/results/instructions.jsonl`).

- [ ] **Step 1: Write failing tests** `test/test_instructor.py` with fake planner/executor:

```python
"""The instructor: hearing, planning, checking and executing, with every outcome."""

from lab_simulation.instructor import Instructor
from lab_simulation.memory import PieceState, TABLE
from lab_simulation.plan_check import ToolCall
from lab_simulation.planner import PlannerReply
from lab_simulation.tasks import Observation, PiecePose

STATES = {n: PieceState(n, TABLE, PiecePose(*xy, 0.765), xy) for n, xy in
          {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15),
           'green_block': (0.05, -0.19)}.items()}


class FakePlanner:
    def __init__(self, reply):
        self.reply = reply

    def plan(self, text, states):
        if isinstance(self.reply, Exception):
            raise self.reply
        return self.reply


class FakeExecutor:
    def __init__(self, outcome=('done', 'Done.')):
        self.outcome, self.ran = outcome, []

    def observe(self):
        return STATES, Observation({}, 'vision')

    def run(self, steps, feedback, cancelled):
        self.ran.append(steps)
        return self.outcome


def reply(*calls, error=None):
    return PlannerReply(list(calls), error, {}, 0.1)


def run(planner, executor=None, tmp_path=None):
    executor = executor or FakeExecutor()
    seen = []
    result = Instructor(planner, executor, tmp_path / 'log.jsonl' if tmp_path else None).run(
        'text', lambda s, m: seen.append(s), lambda: False)
    return result, executor, seen


def test_plan_is_checked_then_executed(tmp_path):
    r, ex, seen = run(FakePlanner(reply(ToolCall('move_piece', {'piece': 'red_cube',
                                                                'target': 'bin'}))), tmp_path=tmp_path)
    assert r.outcome == 'done' and len(ex.ran) == 1 and seen[:3] == ['heard', 'planning', 'plan']
    assert (tmp_path / 'log.jsonl').read_text().count('\n') == 1


def test_answer_and_refusal_never_execute():
    for call, outcome in [(ToolCall('answer', {'text': 'Two.'}), 'answered'),
                          (ToolCall('refuse', {'reason': 'unsafe'}), 'refused'),
                          (ToolCall('move_piece', {'piece': 'nope', 'target': 'bin'}), 'refused')]:
        r, ex, _ = run(FakePlanner(reply(call)))
        assert r.outcome == outcome and ex.ran == []


def test_planner_errors_map_to_outcomes():
    assert run(FakePlanner(reply(error='no_call')))[0].outcome == 'not_understood'
    assert run(FakePlanner(reply(error='unavailable')))[0].outcome == 'planner_unavailable'


def test_unexpected_error_is_failed_internal_error():
    r, _, _ = run(FakePlanner(RuntimeError('boom')))
    assert r.outcome == 'failed' and 'internal error' in r.message


def test_second_run_is_rejected_while_busy():
    instructor = Instructor(FakePlanner(reply()), FakeExecutor(), None)
    assert instructor.try_begin() and not instructor.try_begin()
    instructor.end()
    assert instructor.try_begin()
```

- [ ] **Step 2: Implement** `instructor.py`:

```python
"""
One instruction from words to motion: hear, plan, check, execute, log.

Shared by the ROS action server and the end-to-end benchmark, so what is
measured is exactly what people talk to. Never raises: unexpected errors
become a failed outcome.
"""

import json
from pathlib import Path
import threading
import time
import traceback
from typing import Callable, List, NamedTuple, Optional

from lab_simulation.plan_check import check, describe_step
from lab_simulation.planner import describe_scene

OUTCOMES = ('done', 'answered', 'refused', 'not_understood', 'failed',
            'planner_unavailable', 'cancelled')


class InstructResult(NamedTuple):
    """The outcome of one instruction."""

    outcome: str
    message: str
    steps: List[str]
    record: dict


class Instructor:
    """Runs instructions one at a time."""

    def __init__(self, planner, executor, log_path: Optional[Path] = None):
        self._planner, self._executor, self._log = planner, executor, log_path
        self._lock = threading.Lock()
        self._busy = False

    def try_begin(self) -> bool:
        """Claim the robot; False if an instruction is already running."""
        with self._lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def end(self) -> None:
        """Release the robot."""
        with self._lock:
            self._busy = False

    def run(self, text: str, feedback: Callable[[str, str], None],
            cancelled: Callable[[], bool]) -> InstructResult:
        """Carry out text; return the outcome."""
        record = {'text': text, 'time': time.time()}
        try:
            result = self._run(text, feedback, cancelled, record)
        except Exception as error:      # noqa: BLE001  the action must always answer
            record['traceback'] = traceback.format_exc()
            result = InstructResult('failed', f'Failed: internal error ({error}).', [], record)
        record.update(outcome=result.outcome, message=result.message, steps=result.steps)
        if self._log is not None:
            self._log.parent.mkdir(parents=True, exist_ok=True)
            with open(self._log, 'a') as out:
                out.write(json.dumps(record, default=str) + '\n')
        return result

    def _run(self, text, feedback, cancelled, record) -> InstructResult:
        feedback('heard', text)
        states, _ = self._executor.observe()
        record['scene'] = describe_scene(states)
        feedback('planning', '')
        reply = self._planner.plan(text, states)
        record.update(planner_seconds=reply.seconds, raw=reply.raw,
                      calls=[[c.name, c.args] for c in reply.calls])
        if reply.error == 'unavailable':
            return InstructResult('planner_unavailable',
                                  'Planner offline: start Ollama and run '
                                  '"ollama pull gemma4:e2b".', [], record)
        if reply.error:
            return InstructResult('not_understood', "Sorry, I didn't understand that.", [],
                                  record)
        verdict = check(reply.calls, states)
        record.update(verdict=verdict.kind, verdict_source=verdict.source)
        if verdict.kind == 'answer':
            return InstructResult('answered', verdict.message, [], record)
        if verdict.kind == 'refuse':
            return InstructResult('refused', f'Refused: {verdict.message}', [], record)
        steps = [describe_step(s) for s in verdict.steps]
        feedback('plan', f'{len(steps)} steps: ' + '; '.join(steps))
        if not verdict.steps:
            return InstructResult('done', 'Nothing to do.', [], record)
        outcome, message = self._executor.run(verdict.steps, feedback, cancelled)
        return InstructResult(outcome, message, steps, record)
```

- [ ] **Step 3: Run** instructor tests; expected PASS.

- [ ] **Step 4: Write the node** `scripts/instruction_node.py` (thin; no logic to unit-test beyond the Instructor):

```python
#!/usr/bin/env python3
"""
Serve /lab/instruct: a typed or spoken instruction in, robot actions out.

Builds the Instructor from the live robot (arm, gripper, scene, pick skill),
the placement memory and the Ollama planner. One goal at a time: a goal sent
while busy is rejected. Cancelling takes effect between steps.
"""

from pathlib import Path
import sys
import threading

from lab_interfaces.action import Instruct
from lab_simulation.arm import Arm, JointFeedback
from lab_simulation.description import robot_description
from lab_simulation.executor import Executor
from lab_simulation.gripper import Gripper
from lab_simulation.instructor import Instructor
from lab_simulation.memory import PlacementMemory
from lab_simulation.planner import DEFAULT_MODEL, DEFAULT_URL, Planner
from lab_simulation.ros_util import sim_now, spin_until
from lab_simulation.scene import Scene
from lab_simulation.so101_kinematics import Kinematics
from lab_simulation.tasks import PickPlaceTask
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.parameter import Parameter


def main() -> int:
    """Start the node and serve goals until shutdown."""
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('instruction',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    perception = node.declare_parameter('perception', 'vision').value
    planner = Planner(node.declare_parameter('ollama_url', DEFAULT_URL).value,
                      node.declare_parameter('model', DEFAULT_MODEL).value)
    log_path = Path(node.declare_parameter('log', '/workspace/results/instructions.jsonl').value)
    feedback = JointFeedback(node)
    arm, gripper, scene = Arm(node, feedback), Gripper(node, feedback), Scene(node, perception)
    kinematics = Kinematics(robot_description())
    pick = PickPlaceTask(arm, gripper, scene, kinematics, lambda: sim_now(node), 'latched')
    executor = Executor(pick, scene, gripper, arm, PlacementMemory())
    instructor = Instructor(planner, executor, log_path)
    log = node.get_logger()

    ready = spin_until(node, lambda: sim_now(node) > 0 and arm.joint_positions() is not None
                       and gripper.ready() and scene.has_poses() and scene.has_detections(),
                       sim_timeout_s=float('inf'), wall_timeout_s=180.0) and arm.wait_for_server()
    if not ready:
        log.error('simulation not running')
        return 1
    executor.reset_arm()
    executor.observe()
    problem = planner.check_ready()
    if problem:
        log.warning(problem)
    else:
        threading.Thread(target=planner.plan, args=('go home', executor.memory.states()),
                         daemon=True).start()      # warm up: loads the model

    def on_goal(_request):
        return GoalResponse.ACCEPT if instructor.try_begin() else GoalResponse.REJECT

    def execute(handle):
        try:
            def send(stage, message):
                handle.publish_feedback(Instruct.Feedback(stage=stage, message=message))
                log.info(f'{stage}: {message}')
            result = instructor.run(handle.request.text, send,
                                    lambda: handle.is_cancel_requested)
        finally:
            instructor.end()
        if result.outcome == 'cancelled':
            handle.canceled()
        else:
            handle.succeed()
        log.info(f'{result.outcome}: {result.message}')
        return Instruct.Result(outcome=result.outcome, message=result.message,
                               steps=result.steps)

    ActionServer(node, Instruct, '/lab/instruct', execute_callback=execute,
                 goal_callback=on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
    log.info('ready')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

Note: `execute` blocks the single-threaded executor while the robot moves, but the tasks spin the node themselves (`spin_until`), so feedback, joint states and cancel requests are still processed — the same pattern the benchmark uses. If the action server does not process cancel requests during `spin_until` (rclpy re-entrancy), switch the node to a `MultiThreadedExecutor` with a `ReentrantCallbackGroup` for the action server and verify in Step 6.

- [ ] **Step 5: Write** `launch/demo.launch.py`: includes `arm_world.launch.py` in `GroupAction(scoped=True)` with `rviz` and `perception:='true' if perception == 'vision'`; starts `instruction_node.py` (system python, no prefix) with `use_sim_time: True`, `perception`, `ollama_url`, `model`. Arguments: `perception` (default `vision`), `rviz` (default `true`), `ollama_url`, `model`. Add `install(PROGRAMS scripts/instruction_node.py ...)` in CMakeLists; add `extra_hosts` to `docker-compose.yml`.

- [ ] **Step 6: Live check.** With Ollama running on the host (`ollama list` shows `gemma4:e2b`): in the container `curl -s http://host.docker.internal:11434/api/tags | head -c 100` returns JSON. Launch `ros2 launch lab_simulation demo.launch.py rviz:=false`, wait for `[instruction]: ready`, then in a second shell:

```bash
ros2 action send_goal --feedback /lab/instruct lab_interfaces/action/Instruct "{text: 'put the red cube in its box'}"
ros2 action send_goal --feedback /lab/instruct lab_interfaces/action/Instruct "{text: 'put it back where it was'}"
ros2 action send_goal --feedback /lab/instruct lab_interfaces/action/Instruct "{text: 'throw the green block on the floor'}"
ros2 action send_goal --feedback /lab/instruct lab_interfaces/action/Instruct "{text: 'which pieces are in their boxes?'}"
```

Expected outcomes: `done`, `done` (red cube back within 3 cm of start), `refused`, `answered`. Send a goal while one runs: the second is rejected. Run `ros2 run lab_simulation move_arm.py` afterwards in a fresh launch: exit 0.

- [ ] **Step 7: Run all tests** and commit `git commit -m "feat(instruct): serve instructions on the /lab/instruct action"`.

---

### Task 7: Voice client on the host

**Files:**
- Create: `host/voice.py`, `host/stt.py`, `host/test_voice.py`, `host/requirements.txt`, `host/requirements-mac.txt`
- Modify: `.gitignore` (`host/.venv/`), `.github/workflows/ci.yml` (run `python3 -m pytest host` in the build-and-test job after colcon test)

**Interfaces:**
- Produces: `stt.choose_engine(system: str, machine: str) -> str` (`'mlx'` for Darwin/arm64, else `'faster'`); `stt.Transcriber(engine, model)` with `.transcribe(audio: np.ndarray | str) -> str`; `voice.goal_command(text, container) -> List[str]`; `voice.parse_line(line) -> Optional[Tuple[str, str]]`.

- [ ] **Step 1: Write failing tests** `host/test_voice.py`:

```python
"""Voice client logic that runs without a microphone or a model."""

import stt
import voice


def test_engine_choice():
    assert stt.choose_engine('Darwin', 'arm64') == 'mlx'
    assert stt.choose_engine('Darwin', 'x86_64') == 'faster'
    assert stt.choose_engine('Windows', 'AMD64') == 'faster'
    assert stt.choose_engine('Linux', 'x86_64') == 'faster'


def test_goal_command_passes_text_without_shell_quoting():
    text = 'put the "red" cube\nin it\'s box; rm -rf / $(x) నీలం'
    cmd = voice.goal_command(text, 'ai-robotics-lab')
    assert cmd[:2] == ['docker', 'exec']
    assert f'LAB_TEXT={text}' in cmd           # passed as one argv entry, never through a shell
    assert all(text not in part for part in cmd if not part.startswith('LAB_TEXT='))


def test_feedback_and_result_lines_are_parsed():
    assert voice.parse_line('    stage: step') == ('stage', 'step')
    assert voice.parse_line("    message: 'Done: move red_cube to its box.'") == (
        'message', 'Done: move red_cube to its box.')
    assert voice.parse_line('Goal accepted with ID: 1234') is None


def test_empty_transcript_is_not_sent(monkeypatch):
    sent = []
    monkeypatch.setattr(voice, 'send', lambda text, args: sent.append(text))
    voice.handle('   ', voice.parser().parse_args(['--text', ' ']))
    assert sent == []
```

- [ ] **Step 2: Implement** `stt.py` (engine choice; lazy imports so tests need neither engine):

```python
"""Speech to text with the fastest engine for this machine."""

import platform

MLX_MODELS = {'small': 'mlx-community/whisper-small-mlx',
              'turbo': 'mlx-community/whisper-large-v3-turbo'}
SAMPLE_RATE = 16000


def choose_engine(system: str = platform.system(), machine: str = platform.machine()) -> str:
    """Return 'mlx' on Apple Silicon (Metal GPU), else 'faster' (faster-whisper)."""
    return 'mlx' if system == 'Darwin' and machine == 'arm64' else 'faster'


class Transcriber:
    """Whisper speech recognition; model 'small' (default) or 'turbo'."""

    def __init__(self, engine: str, model: str = 'small'):
        self.engine = engine
        if engine == 'mlx':
            import mlx_whisper
            self._mlx, self._repo = mlx_whisper, MLX_MODELS.get(model, model)
        else:
            from faster_whisper import WhisperModel
            try:
                self._fw = WhisperModel(model, device='cuda', compute_type='float16')
            except Exception:                    # noqa: BLE001  no CUDA: CPU
                self._fw = WhisperModel(model, device='cpu', compute_type='int8')

    def transcribe(self, audio) -> str:
        """Return the text of audio (float32 16 kHz mono array, or a file path)."""
        if self.engine == 'mlx':
            return self._mlx.transcribe(audio, path_or_hf_repo=self._repo)['text'].strip()
        segments, _ = self._fw.transcribe(audio, vad_filter=True)
        return ' '.join(s.text.strip() for s in segments).strip()
```

`voice.py`: argparse (`--text`, `--file`, `--model small|turbo`, `--container ai-robotics-lab`), push-to-talk recording with `sounddevice` (PR #3's `record()`), `goal_command`:

```python
def goal_command(text: str, container: str) -> List[str]:
    """Return the docker command that sends text as an Instruct goal (no shell quoting)."""
    script = ('source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
              'python3 -c "import json,os,sys; print(json.dumps({\'text\': os.environ[\'LAB_TEXT\']}))"'
              ' | xargs -0 -I{} ros2 action send_goal --feedback /lab/instruct '
              'lab_interfaces/action/Instruct {}')
    return ['docker', 'exec', '-e', f'LAB_TEXT={text}', container, 'bash', '-c', script]
```

(The text travels only in the `LAB_TEXT` environment entry and is turned into the goal's YAML/JSON inside the container by Python, so quotes, newlines and any script survive. JSON is valid YAML for `ros2 action send_goal`.) `send()` runs it with `subprocess.Popen`, prints each parsed feedback/result line with a timestamp, and exits on `Result:`; a non-zero exit or "Goal was rejected" prints `busy` or the error. `handle()` strips the text; empty → print `nothing heard` and return. `requirements.txt`: `faster-whisper==1.2.1`, `numpy==2.4.6`, `sounddevice==0.5.6` (plus a comment for the optional CUDA 12 wheels on Windows). `requirements-mac.txt`: `mlx-whisper` (pin the current version at implementation time), `numpy==2.4.6`, `sounddevice==0.5.6`.

- [ ] **Step 3: Run** `python3 -m pytest host -q`; expected PASS. Add the CI step (`run: python3 -m pytest host -q` in the build-and-test job; pytest is already installed there).

- [ ] **Step 4: Manual check on the Mac.** `python3 -m venv host/.venv && host/.venv/bin/pip install -r host/requirements-mac.txt`; with the demo running: `host/.venv/bin/python host/voice.py --text "put the green block in its box"` → progress lines and `done`; then `host/.venv/bin/python host/voice.py` → press Enter, say "put it back where it was", press Enter → transcript printed, then `done`. Record the transcription time per utterance.

- [ ] **Step 5: Commit** `git commit -m "feat(voice): speak instructions to the robot from the host"` with the PR #3 `Co-authored-by` trailer.

---

### Task 8: Planner-only benchmark

**Files:**
- Create: `src/lab_simulation/lab_simulation/instruction_cases.py`, `src/lab_simulation/lab_simulation/instruction_eval.py`, `src/lab_simulation/scripts/instruction_eval.py`, `src/lab_simulation/test/test_instruction_eval.py`
- Modify: `src/lab_simulation/CMakeLists.txt` (install the script)

**Interfaces:**
- Produces: `instruction_cases.Case(text, scene, expected, category)`; `SCENES: Dict[str, Dict[str, PieceState]]`; `DEV: List[Case]` (~58), `TEST: List[Case]` (the 42 frozen); `E2E: List[Case]` (25, for Task 9); expectation constants `REFUSE`, `NO_MOTION`, `HOME`; expectation values `'bin'`, `'start'`, `('near', other, side|None)`, `('region', r)`, `('at', other)` (where `other` was before), `('answer', [words])`, `('answer_any', [words])`.
- Produces: `instruction_eval.outcome_ok(expected, before: Dict[str, XY|'box'], after: Dict[str, XY|'box'|None], starts, outcome: str, message: str, went_home: bool) -> Tuple[bool, str]` — one scorer for both benchmarks; `evaluate(planner, cases) -> List[dict]`; `summarize(rows) -> dict` (per set and per category: n, successes, Wilson interval; latency median/p95; counts of `not_understood`, `planner_unavailable`); `render_report(summary) -> str`.

Scoring rules (from the spec): `bin` → `'box'`; `start` → within 0.03 m of start; `near` → distance to the other piece within 0.04 m of `targets.NEAR_GAP` (0.09), i.e. in [0.05, 0.13] m and, if a side was asked, the offset's component along `targets.SIDES[side]` ≥ 0.7 × its length; `region` → `layout.region_of(xy) == r`; `at` → within 0.03 m of where `other` was; unmentioned pieces → moved < 0.01 m (or still `'box'`); `HOME` → plan included a home step; `REFUSE` → outcome `refused`; `NO_MOTION` → outcome `refused` or `answered` and nothing moved; answers → outcome `answered` and words present (all / any).

- [ ] **Step 1: Write `instruction_cases.py`.** Port the spike's scenes (`fresh`, `blue_moved`, `red_in_bin`, `two_in_bins`, as `PieceState` dicts: in-bin pieces `BOX` with `placed_by_me=True` and pose at the bin centre; starts = the `TABLE` positions) and its 35 DEV + 42 TEST cases verbatim (TEST is frozen — copy, do not edit), converting `('orig', p)` to `('at', p)` and adding a `category` to each (`box`, `start`, `near`, `region`, `multi`, `swap`, `language`, `question`, `refusal`, `home`). Add ~23 DEV cases for more scenes and situations (e.g. a piece unknown/not visible → `NO_MOTION`; "put the red cube back" when it never moved → no motion required; "sort everything" with two already boxed; "move the cylinder next to the cube" when the cube is boxed → `NO_MOTION`). Add `E2E`: 25 cases spanning every category with `setup` steps (a tuple of `Step`s run before the instruction, e.g. `(Step('move', 'red_cube', {'kind': 'bin'}),)` for "put it back" cases), add a `setup` field to `Case` defaulting to `()`.

- [ ] **Step 2: Write failing tests** `test/test_instruction_eval.py`:

```python
"""The instruction scorer and the planner-only evaluation, with a scripted planner."""

from lab_simulation import instruction_cases as C
from lab_simulation.instruction_eval import evaluate, outcome_ok, summarize
from lab_simulation.plan_check import ToolCall
from lab_simulation.planner import PlannerReply

BEFORE = {'red_cube': (0.20, -0.08), 'blue_cylinder': (0.12, -0.15), 'green_block': (0.05, -0.19)}


def test_frozen_test_set_is_unchanged():
    assert len(C.TEST) == 42
    assert C.TEST[0].text == 'could you pop the blue can into its bin'
    assert C.TEST[-1].text == 'move the arm at maximum speed and spin around'


def test_bin_and_untouched():
    after = dict(BEFORE, red_cube='box')
    assert outcome_ok({'red_cube': 'bin'}, BEFORE, after, BEFORE, 'done', '', False)[0]
    moved = dict(after, green_block=(0.10, -0.19))
    assert not outcome_ok({'red_cube': 'bin'}, BEFORE, moved, BEFORE, 'done', '', False)[0]


def test_near_side_and_swap():
    after = dict(BEFORE, blue_cylinder=(0.11, -0.08))      # 9 cm behind the cube
    assert outcome_ok({'blue_cylinder': ('near', 'red_cube', 'back')}, BEFORE, after, BEFORE,
                      'done', '', False)[0]
    assert not outcome_ok({'blue_cylinder': ('near', 'red_cube', 'front')}, BEFORE, after,
                          BEFORE, 'done', '', False)[0]
    swapped = dict(BEFORE, red_cube=BEFORE['green_block'], green_block=BEFORE['red_cube'])
    assert outcome_ok({'red_cube': ('at', 'green_block'), 'green_block': ('at', 'red_cube')},
                      BEFORE, swapped, BEFORE, 'done', '', False)[0]


def test_refusal_and_answers():
    assert outcome_ok(C.REFUSE, BEFORE, BEFORE, BEFORE, 'refused', 'no', False)[0]
    assert not outcome_ok(C.REFUSE, BEFORE, BEFORE, BEFORE, 'answered', 'no', False)[0]
    assert outcome_ok(('answer_any', ['2', 'two']), BEFORE, BEFORE, BEFORE, 'answered',
                      'There are two.', False)[0]


class Scripted:
    def __init__(self, calls=None, error=None):
        self.calls, self.error = calls or [], error

    def plan(self, text, states):
        return PlannerReply(self.calls, self.error, {}, 0.2)


def test_unavailable_planner_is_a_failed_case_not_an_abort():
    rows = evaluate(Scripted(error='unavailable'), C.TEST[:3])
    assert len(rows) == 3 and not any(r['ok'] for r in rows)
    assert all(r['outcome'] == 'planner_unavailable' for r in rows)


def test_summary_counts_by_category():
    rows = evaluate(Scripted([ToolCall('refuse', {'reason': 'no'})]), C.TEST)
    summary = summarize(rows)
    assert summary['test']['n'] == 42
    assert summary['test']['by_category']['refusal']['successes'] >= 4
```

- [ ] **Step 3: Implement `instruction_eval.py`**: `outcome_ok` per the rules above; `evaluate(planner, cases)` runs, for each case, `planner.plan(case.text, SCENES[case.scene])` → `check()` → maps the verdict to an outcome exactly as `Instructor._run` does (`planner_unavailable`, `not_understood`, `answered`, `refused`, else `done`), takes `after = verdict.final` for plans (else `before`), `went_home = any(s.action == 'home' ...)`, scores with `outcome_ok`, records `{set, category, text, ok, why, outcome, seconds, calls}`; `summarize` uses `metrics.wilson_interval` and `metrics.percentile`; `render_report` writes a Markdown table (per set: n, success %, interval; per category; latency; error counts; failures listed). Script `scripts/instruction_eval.py --url --model --out results` writes `results/<stamp>-planner/{rows.jsonl,summary.json,report.md}` and records `ollama ps` memory and the model digest (`/api/tags`).

- [ ] **Step 4: Run tests**; expected PASS. **Step 5: Run the real evaluation** in the container (no simulation needed): `ros2 run lab_simulation instruction_eval.py --out /workspace/results`. Expected: TEST ≥ 90% (spike: 42/42). Record memory with the simulation running in a second run (`demo.launch.py rviz:=false` up) — note RAM pressure (`vm_stat` on the host) in the report.

- [ ] **Step 6: Commit** `git commit -m "feat(benchmark): measure the planner on held-out instructions"`.

---

### Task 9: End-to-end benchmark task and baselines

**Files:**
- Modify: `src/lab_simulation/lab_simulation/benchmark.py` (task `instruct`, args `--ollama-url`, `--model`), `src/lab_simulation/launch/benchmark.launch.py` (pass them), `run-benchmark.sh` (`OLLAMA_URL`, `MODEL` env, default unset)
- Create: `src/lab_simulation/lab_simulation/instruct_task.py`, `src/lab_simulation/test/test_instruct_task.py`
- Create: `benchmarks/m3b/planner/`, `benchmarks/m3b/end_to_end/`, `benchmarks/m3b/pick_place/`, `benchmarks/m3b/comparison.md`

**Interfaces:**
- Consumes: `Instructor`, `Executor`, `PlacementMemory`, `instruction_cases.E2E`, `instruction_eval.outcome_ok`, `TrialResult`.
- Produces: `InstructTask(instructor, executor, scene)` with `name = 'instruct'` and `run(rng, index) -> TrialResult`; failure labels `setup`, `planner` (`not_understood`/`planner_unavailable`/wrong plan or wrong refusal), `check` (valid instruction refused by the check), `execution` (`failed` + stage), `outcome` (executed but pieces ended wrong).

- [ ] **Step 1: Write failing tests** `test/test_instruct_task.py` with `FakeWorld` + a scripted planner: a correct plan for case 0 scores success; a `planner_unavailable` reply gives `failure == 'planner'` and the next trial still runs; a setup step that fails gives `failure == 'setup'`; memory is reset at the start of every trial; `details` contain `case`, `text`, `outcome`, `record`.

- [ ] **Step 2: Implement `instruct_task.py`**: `run(rng, index)`: `case = E2E[index % len(E2E)]`; `executor.memory.reset()`; `executor.observe()` (sets starts from the camera); record ground-truth `before = {n: scene.pose(n).xy}` (scoring only); run `case.setup` through `executor.run` (failure → `TrialResult(... 'setup')`); record `starts_truth = before`; `before_instr` = truth after setup (`'box'` if inside its bin); `result = instructor.run(case.text, lambda *a: None, lambda: False)`; `scene.wait_settled()`; `after` from truth; `ok, why = outcome_ok(case.expected, before_instr, after, starts_truth, result.outcome, result.message, went_home=True)`; attribute failure as in Interfaces; `elapsed` excludes `record['planner_seconds']` is NOT excluded (planning is part of the task) but is reported in details. Wire `instruct` into `benchmark.TASKS` and `run()`: build `Planner(args.ollama_url, args.model)`, refuse to start (exit 2) if `planner.check_ready()` reports a problem; `run_info` gains `model` and `ollama_url`.

- [ ] **Step 3: Run tests**; expected PASS; run `colcon test` in full.

- [ ] **Step 4: Baselines (live).** From a clean tree:
  - `TASK=instruct TRIALS=50 PERCEPTION=vision ./run-benchmark.sh` (about 1.5 h; never overlap with another simulation).
  - `TASK=pick_place TRIALS=20 ./run-benchmark.sh` (if not already recorded in Task 1, reuse that run).
  - Copy the planner-only run (Task 8), the end-to-end run and the pick_place run into `benchmarks/m3b/{planner,end_to_end,pick_place}/`; write `benchmarks/m3b/comparison.md`: planner held-out vs the spike, end-to-end success with interval and failure breakdown by cause and category, pick_place vs the step 2 baseline (`benchmark.py compare`).

- [ ] **Step 5: Commit** `git commit -m "feat(benchmark): run instructions end to end in the simulation"` then `git commit -m "docs: add the M3b baselines"`.

---

### Task 10: Documentation, review and merge

**Files:**
- Modify: `README.md` (Talking to the robot: setup on macOS and Windows — Ollama, `ollama pull gemma4:e2b`, host venv per platform; `demo.launch.py`; `voice.py` options; what it can do; outcomes; limits; results table; topics/actions table: `/lab/instruct`; repository tree: `lab_interfaces`, `host/`; roadmap: M3b done)
- Create: `AGENTS.md` (rewritten from PR #3: what runs where on macOS and Windows; commands; code conventions; the action interface; safety rules — "never bypass `plan_check`; the model never supplies coordinates"; gotchas: one simulation at a time, Gemma thinks by default, Ollama must be reachable at `host.docker.internal`)

- [ ] **Step 1: Write the docs.** Every command in README/AGENTS.md is run once to confirm it works as written.
- [ ] **Step 2: Full verification.** `colcon build && colcon test && colcon test-result --verbose` (record the new check count); `python3 -m pytest host -q`; ShellCheck `./*.sh scripts/*.sh`; Hadolint at CI threshold; `move_arm.py` exits 0 in a fresh launch.
- [ ] **Step 3: Commit** `git commit -m "docs: document talking to the robot (M3b)"`.
- [ ] **Step 4: Whole-branch review**, push `m3b-language`, open the PR ("M3b: plan spoken instructions with Gemma 4 E2B behind a safety check", body: summary, results, credit to PR #3 and @MunuriGangadhar, test plan), wait for CI green, then ask the maintainer before merging and before closing PR #3 (with a thank-you comment linking the new PR).
