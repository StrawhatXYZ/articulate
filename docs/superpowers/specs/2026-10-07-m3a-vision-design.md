# M3a: Vision — Sorting From a Camera

- **Status:** Draft, awaiting review
- **Date:** 7 October 2026
- **Roadmap position:** Step 3, first of three AI milestones (M3a vision, M3b language, M4 learning)

## 1. Context: the AI integration plan

Agreed approach (staged hybrid):

1. **M3a Vision** (this spec): the robot finds and names objects from a camera
   instead of reading their positions from the simulator.
2. **M3b Language**: typed, then spoken, instructions are turned into robot
   skills by a language-model planner, behind a safety gate.
3. **M4 Learning**: the scripted pipeline becomes a data engine for
   demonstrations; learned policies (ACT, then SmolVLA) are trained on them and
   compared with the scripted skills on the benchmark.

Each milestone is measured on the existing seeded benchmark.

## 2. Purpose

Today every task plans from ground-truth piece poses published by Gazebo. M3a
makes the robot plan only from what an overhead RGB-D camera shows, with
open-vocabulary naming, and measures what that costs against the 93%
ground-truth sorting baseline.

Success means:

1. Sorting runs with planning driven only by camera detections; ground truth
   is used only to score outcomes.
2. Detection quality is reported per run: naming accuracy, position error,
   missed and false detections.
3. A 100-trial vision-mode baseline is committed and compared with the
   ground-truth baseline.

Out of scope: language instructions (M3b), the wrist camera and learned
policies (M4), cluttered or touching pieces, detecting the bins (they are
fixed, like a real work cell).

## 3. Decisions

| Topic | Decision | Rejected alternative |
|---|---|---|
| Camera | One fixed overhead RGB-D camera | Wrist camera (M4); both now (extra rendering cost) |
| Finding objects | Depth: pixels more than 5 mm above the known table plane, grouped into blobs | Open-vocabulary detector (OWLv2, Grounding DINO): 1–5 s per frame on CPU; colour thresholds: brittle, not AI |
| Naming objects | Embedding model matches each blob's colour crop to text labels; backend swappable (EmbeddingGemma 2 or SigLIP, chosen by the spike) | Training a classifier (no open vocabulary) |
| Where models run | CPU inside the lab container | Native macOS with GPU (needs a host–container bridge) |
| Output | `vision_msgs/Detection3DArray` on `/lab/perception/objects` | Custom message type |
| When to look | Before each pick, from the arm-clear home pose, on a frame taken after pieces settle | Continuous tracking (occluded by the arm) |

## 4. Spike (before the implementation plan)

The spike answers, with numbers recorded in the plan:

1. **Rendering cost:** real-time factor of the benchmark with the camera at
   640×480, 10 Hz, against today's 3.3×. If it falls below 1×, reduce
   resolution or render on demand.
2. **Occlusion:** a camera pose from which the home-pose arm hides no part of
   the workspace.
3. **CI:** whether the camera renders on GitHub's headless runners. If not, CI
   runs ground-truth mode only and vision mode is verified locally.
4. **Naming backend:** EmbeddingGemma 2 against SigLIP on CPU latency per crop
   and naming accuracy over crops of the three pieces under randomised
   lighting. The more accurate one becomes the default; a tie goes to the
   faster.

Model weights are downloaded into a cached Docker volume, not baked into the
image.

## 5. Design

### 5.1 Camera

A Gazebo RGB-D sensor on a static mount above the table, pose chosen by the
spike. It is bridged to `/lab/camera/image`, `/lab/camera/depth` and
`/lab/camera/camera_info`, and shown in RViz.

### 5.2 Package `lab_perception`

Pure-Python core (NumPy, no ROS) with a thin ROS node.

- `segmentation.py`: depth image + camera intrinsics + camera pose → blobs.
  Each blob carries its 3D centroid on the table, footprint, top height and a
  colour crop. Blobs smaller than a minimum area are dropped.
- `naming.py`: `Namer` interface; `name(crop) -> (label, score)`. Labels come
  from text prompts ("a red cube", "a blue cylinder", "a green block"). A score
  under the confidence threshold yields `unknown`. Backends: EmbeddingGemma 2,
  SigLIP, and a deterministic fake for tests.
- `perception_node.py`: synchronises colour and depth, runs segmentation and
  naming, publishes `Detection3DArray` (label, score, 3D pose).

### 5.3 Integration with the tasks

- `Scene` gains a pose source: `ground_truth` (today) or `vision`.
- In vision mode, `Scene.observe()` waits for a frame stamped after the scene
  settled with the arm at home, and returns the detected pieces. Tasks plan
  only from those.
- Outcome scoring (placement, in-bin, upright, disturbance) always uses ground
  truth.
- The benchmark gains `--perception {ground_truth,vision}`; run metadata
  records it, and comparisons across modes carry a warning.

### 5.4 Randomisation

Per trial, from the trial's seeded stream: sun direction and intensity, and
each piece's starting yaw. Positions stay as today. Ground-truth and vision
runs with the same seed see the same scenes.

## 6. Error handling

- Piece not detected → trial failure `not_found` (new).
- Blob below the confidence threshold → `unknown`, never picked.
- Two blobs with the same label → the higher score is used; the other is
  logged in the trial details.
- No camera frames or perception node not running → benchmark exits 2 (no
  simulation), as today.
- Wrong position → surfaces as `grasp_miss` and is counted.

## 7. Measurement

Added to the summary and report for vision-mode runs:

| Metric | Definition |
|---|---|
| Naming accuracy | Detections whose label matches the piece whose true centre is nearest (within 3 cm) |
| Position error | Distance between detected and true centre, matched detections, mm (mean, median, p95) |
| Missed | Pieces with no detection within 3 cm |
| False | Detections with no piece within 3 cm |

The headline comparison is sorting success, vision against ground truth, same
seed and trials.

## 8. Testing

- Segmentation: unit tests on synthetic depth images rendered from boxes and
  cylinders through the real camera model, with known centres (position error
  under 5 mm), plus missing-data and touching-edge cases.
- Naming: tests with the fake backend; one real-model test, skipped when the
  weights are absent.
- Integration: `Scene` vision mode with recorded frames; a 5-trial vision-mode
  benchmark locally (and in CI if the spike allows).

## 9. Success criteria

1. `benchmark.launch.py task:=sort perception:=vision rviz:=true realtime:=true`
   sorts from the camera, with the camera view visible in RViz.
2. A 100-trial vision-mode sorting baseline and its comparison with ground
   truth are committed.
3. All tests pass; CI green; README documents vision mode and the numbers.
