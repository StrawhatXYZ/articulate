# Step 2 baseline: SO-101 compared with step 1

Seed 42, 100 trials per task, commit `6526dbd`, native arm64 (Apple Silicon).
Generated with `benchmark.py compare`; the warnings are expected, since the
robot and grasp differ by design.

- `step1-native`: [`../baseline/native`](../baseline/native), the 3-DOF arm
  with suction (step 1).
- `step2-latched`: [`latched`](latched), the SO-101 gripping with its jaws, with
  the grasp latch (default mode).
- `step2-friction`: [`friction`](friction), the SO-101 holding pieces by
  friction alone (pick and place only).

## Step 1 to step 2

> Warning: different robot: simple_arm vs so101
> Warning: different grasp mode: suction vs latched

| Metric | step1-native | step2-latched | Change | Ratio | Verdict |
|---|---|---|---|---|---|
| pick_place success rate | 0.840 | 1.000 | 0.160 | 1.19× | better |
| pick_place cycle time mean, successes (s) | 15.564 | 12.119 | -3.446 | 0.78× | not tested |
| pick_place error median, measured trials (m) | 0.007 | 0.005 | -0.002 | 0.73× | not tested |
| pick_place error mean, measured trials (m) | 0.011 | 0.006 | -0.005 | 0.53× | not tested |
| reach success rate | 1.000 | 1.000 | 0.000 | 1.00× | not significant |
| reach cycle time mean, successes (s) | 3.483 | 1.267 | -2.216 | 0.36× | not tested |
| reach error median, measured trials (m) | 0.002 | 0.000 | -0.002 | 0.19× | not tested |
| reach error mean, measured trials (m) | 0.002 | 0.000 | -0.002 | 0.18× | not tested |
| real-time factor | 4.437 | 3.687 | -0.750 | 0.83× | not tested |
| trials per minute | 22.205 | 26.607 | 4.403 | 1.20× | not tested |

## Latched compared with friction-only

> Warning: different grasp mode: latched vs friction
> Warning: different task set: ['pick_place', 'reach'] vs ['pick_place']; throughput is not compared

| Metric | step2-latched | step2-friction | Change | Ratio | Verdict |
|---|---|---|---|---|---|
| pick_place success rate | 1.000 | 0.640 | -0.360 | 0.64× | worse |
| pick_place cycle time mean, successes (s) | 12.119 | 11.832 | -0.287 | 0.98× | not tested |
| pick_place error median, measured trials (m) | 0.005 | 0.006 | 0.001 | 1.16× | not tested |
| pick_place error mean, measured trials (m) | 0.006 | 0.007 | 0.001 | 1.20× | not tested |

Friction-only failures are all `dropped`: the piece slips more than 1 cm in
the jaws while carried. By piece: green block 41/44, red cube 18/25,
blue cylinder 5/31. Flat jaws hold the round cylinder by two lines of
contact, and Gazebo lets it slide out. Latched: 44/44, 25/25, 31/31.
