# Benchmarks

Every capability is measured on a seeded, repeatable benchmark; the runs behind each table are committed under [`benchmarks/`](../benchmarks/).

## Benchmark

`run-benchmark.sh` runs seeded trials against a headless simulation and writes
`results/<date-time>/` containing `trials.jsonl`, `summary.json` and
`report.md`.

```bash
./run-benchmark.sh                                  # all three tasks, 100 trials each, seed 42
TASK=sort TRIALS=10 ./run-benchmark.sh              # colour sorting only
TASK=pick_place TRIALS=10 ./run-benchmark.sh        # quick check
TASK=pick_place GRASP_MODE=friction ./run-benchmark.sh   # without the grasp latch
TASK=sort PERCEPTION=vision ./run-benchmark.sh      # plan from the camera
```

| Task | What happens | Success means |
|---|---|---|
| `reach` | The gripper, pointing down, moves to a random point 3–8 cm above the table, clear of the pieces | Grasp point within 1 cm |
| `pick_place` | The jaws close on a random piece; MoveIt carries it to a random target | Jaws closed on the piece, held within 1 cm of its grip while carried, placed within 2 cm of the target, upright (tilt under 15°) on the table |
| `sort` | Every piece, one after another, is put into the box of its colour | All three pieces end upright inside their own boxes (checked at the end, so a piece knocked out later counts) |

Failures are classified as `unreachable`, `planning_failed`, `not_found`,
`grasp_miss`, `dropped`, `placement_error` or `timeout`; each pick also
records the stage it reached. Success rates carry 95% Wilson
intervals. Robot timings are in simulation seconds; trials per minute and
real-time factor are wall-clock. The benchmark lifts Gazebo's real-time cap
unless `realtime:=true`. Every trial randomises the pieces' positions and
angles and the lighting, from the seed.

**Perception modes.** With `perception:=ground_truth` (the default) the
robot plans from the simulator's exact poses. With `perception:=vision` it
plans only from the camera's detections, including where MoveIt's obstacles
are, and looks again before every pick; a piece it cannot see fails as
`not_found`. The true poses are used only to score the outcome. Vision-mode
reports add a detection-quality table (naming accuracy, position error,
missed and false detections, latency); cycle times leave out the time spent
waiting for the camera.

### Step 2 baseline (SO-101)

Seed 42, 100 trials per task, native arm64
([`benchmarks/step2/`](../benchmarks/step2/), with the full comparison):

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
[Grasping, and the grasp latch](architecture.md#grasping-and-the-grasp-latch)): every grasp
was a real jaw closure, but Gazebo's friction alone holds only 73% of pieces
to the end of the carry, and only 10 of 30 cylinders. How well the real
gripper holds is something only hardware can measure.

Step 2's pieces, layout (with the sorting boxes) and reach targets are sized to the SO-101 (reach
targets lie 3–8 cm above the table, step 1's 5–65 cm), so the reach rows
compare each robot on its own workspace.

The step 1 baseline (3-DOF arm, native and emulated images) stays in
[`benchmarks/baseline/`](../benchmarks/baseline/); step 1 itself can be checked
out as tag `v0.2.0`.

### M3a baseline: sorting from the camera

Colour sorting, seed 42, 100 trials each, random positions, angles and
lighting; [`benchmarks/m3a/`](../benchmarks/m3a/) has the reports and the full
comparison.

| | Ground truth (simulator poses) | Vision (camera + SigLIP 2) |
|---|---|---|
| Sorting success (all 3 pieces) | 93% (86–97%) | **92% (85–96%)** |
| Sorting time, mean (excluding camera waits) | 37.2 s | 34.9 s |
| Pieces named correctly | – | 588 / 588 |
| Detection position error, mean / p95 | – | 1.4 / 1.8 mm |
| Missed / false detections | – | 0 / 0 |
| Wait per look | – | 1.2 s |
| Trials per minute | 3.9 | 2.9 |

Planning only from what the camera sees costs one point of success, well
within the statistical noise: perception is not the bottleneck. The main
remaining failure in both modes is the tall cylinder tipping over after it
is released in its box.

### M3b baseline: instructions to robot actions

Gemma 4 E2B on an Apple M4 with the simulation running;
[`benchmarks/m3b/`](../benchmarks/m3b/) has the reports and the full write-up.

| | Result |
|---|---|
| Planner, 42 held-out instructions | 40/42; **40/40** of those the table has room for |
| End to end in the simulation (camera only, 50 runs) | **38/50 (76%, 63–86%)**; 38/46 feasible |
| Unsafe or non-existent requests refused, nothing moved | 6/6 end to end, 8/8 held-out |
| Planning time, median | 6–8 s |
| Instruction time, median (planning + motion) | 26 s |

The language model is rarely the problem: two phrasings ("the red and the
blue ones", "in front of the cube") account for its misses. The weak links
are picking a piece back out of its box from memory (2/6 "put it back"
trials) and the crowded table: a quarter of the misses are instructions with
no room to carry out, which the robot refuses instead of improvising.

```bash
ros2 run lab_simulation instruction_eval.py                 # planner only (needs Ollama)
TASK=instruct TRIALS=50 PERCEPTION=vision ./run-benchmark.sh   # end to end (host)
```

Compare two runs. Success rates are tested for significance (overlapping 95%
intervals read "not significant"); other rows have no test and read
"not tested". Runs with different robots, grasp modes, seeds, trial counts or
task sets are flagged.

```bash
docker exec articulate bash -c 'source /workspace/install/setup.bash && ros2 run lab_simulation benchmark.py compare /workspace/results/A /workspace/results/B'
```
