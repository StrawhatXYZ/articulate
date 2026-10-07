# Step 2 baseline: SO-101 compared with step 1

Seed 42, 100 trials per task, native arm64 (Apple Silicon). Generated with
`benchmark.py compare`; the warnings are expected, since the robot and grasp
differ by design.

- `step1-native`: [`../baseline/native`](../baseline/native), the 3-DOF arm
  with suction (step 1).
- `step2-latched`: [`latched`](latched), the SO-101 gripping with its jaws, with
  the grasp latch (default mode); reach, pick-and-place and colour sorting,
  commit `fd3d5e8`.
- `step2-friction`: [`friction`](friction), the SO-101 holding pieces by
  friction alone; pick-and-place only, commit `9434766` (the later commit
  changed only how pieces are released inside sorting bins).

So do the tasks' geometry: step 2's pieces are sized for the SO-101's jaws,
the layout follows the SO-101's workspace with three sorting bins on its
left, and reach targets lie 3–8 cm above the table (step 1: 5–65 cm). Read
the reach rows as each robot on its own workspace, not like for like. The
sorting task is new in step 2 and has no step 1 counterpart.

## Step 1 to step 2

> Warning: different robot: simple_arm vs so101
> Warning: different grasp mode: suction vs latched
> Warning: different task set: ['pick_place', 'reach'] vs ['pick_place', 'reach', 'sort']; throughput is not compared

| Metric | step1-native | step2-latched | Change | Ratio | Verdict |
|---|---|---|---|---|---|
| pick_place success rate | 0.840 | 0.980 | 0.140 | 1.17× | better |
| pick_place cycle time mean, successes (s) | 15.564 | 11.465 | -4.099 | 0.74× | not tested |
| pick_place error median, measured trials (m) | 0.007 | 0.005 | -0.002 | 0.73× | not tested |
| pick_place error mean, measured trials (m) | 0.011 | 0.007 | -0.004 | 0.62× | not tested |
| reach success rate | 1.000 | 1.000 | 0.000 | 1.00× | not significant |
| reach cycle time mean, successes (s) | 3.483 | 1.296 | -2.187 | 0.37× | not tested |
| reach error median, measured trials (m) | 0.002 | 0.000 | -0.002 | 0.18× | not tested |
| reach error mean, measured trials (m) | 0.002 | 0.000 | -0.002 | 0.18× | not tested |

## Colour sorting (step 2 only)

93/100 trials put all three pieces upright into their own bins (95% CI
86–97%), in 37.6 s per trial on average (three picks). Each of the 7
failed trials left one piece unsorted: the red cube could not be reached at
its start position 3 times, the blue cylinder tipped over in its bin twice,
and a red cube and a green block each timed out once.

## Latched compared with friction-only

> Warning: different grasp mode: latched vs friction
> Warning: different task set: ['pick_place', 'reach', 'sort'] vs ['pick_place']; throughput is not compared

| Metric | step2-latched | step2-friction | Change | Ratio | Verdict |
|---|---|---|---|---|---|
| pick_place success rate | 0.980 | 0.730 | -0.250 | 0.74× | worse |
| pick_place cycle time mean, successes (s) | 11.465 | 11.154 | -0.311 | 0.97× | not tested |
| pick_place error median, measured trials (m) | 0.005 | 0.006 | 0.000 | 1.09× | not tested |
| pick_place error mean, measured trials (m) | 0.007 | 0.006 | -0.000 | 0.99× | not tested |

Friction-only failures are almost all `dropped`: the piece slips more than
1 cm in the jaws while carried. By piece: green block 35/36, red cube 28/34,
blue cylinder 10/30. Flat jaws hold the round cylinder by two lines of
contact, and Gazebo lets it slide out.
