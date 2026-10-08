# M3a baseline: sorting from the camera vs ground truth

Colour sorting (all three pieces into their own boxes), seed 42, 100 trials
each, native arm64 (Apple Silicon), random piece positions, angles and
lighting per trial; both runs see the same scenes.

- [`ground_truth`](ground_truth): the robot plans from the simulator's exact
  poses (commit `bf8b72a`).
- [`vision`](vision): the robot plans only from the overhead RGB-D camera,
  with SigLIP 2 naming the pieces (commit `59edf4b`; the commits between
  change only vision-mode bookkeeping and how a MoveIt planning error is
  labelled).

Generated with `benchmark.py compare`; the warning is expected.

> Warning: different perception: ground_truth vs vision

| Metric | ground_truth | vision | Change | Ratio | Verdict |
|---|---|---|---|---|---|
| sort success rate | 0.930 | 0.920 | -0.010 | 0.99× | not significant |
| sort cycle time mean, successes (s) | 37.191 | 34.947 | -2.243 | 0.94× | not tested |
| sort error median, measured trials (m) | 0.009 | 0.011 | 0.002 | 1.22× | not tested |
| sort error mean, measured trials (m) | 0.010 | 0.013 | 0.003 | 1.29× | not tested |
| real-time factor | 2.507 | 2.068 | -0.439 | 0.82× | not tested |
| trials per minute | 3.865 | 2.907 | -0.958 | 0.75× | not tested |

**Perception costs one point of sorting success, within noise** (93% vs 92%,
95% intervals 86–97% and 85–96%).

## Detection quality (vision run)

| Observations | Pieces named correctly | Missed | False | Position error mean / p95 | Wait per look |
|---|---|---|---|---|---|
| 294 | 588 / 588 (100%) | 0 | 0 | 1.4 / 1.8 mm | 1.2 s (wall) |

Detections are scored against the true poses for pieces on the open table;
pieces already sorted sit in a bin, which perception masks out by design.

## Failures

| | Ground truth | Vision |
|---|---|---|
| Blue cylinder tipped over in its bin (`placement_error`) | 2 | 5 |
| Timeouts while carrying or retreating | 5 | 0 |
| Planning failed on the retreat | 0 | 1 |
| Timeout on the approach | 1 | 1 |
| Scene reset failed (`timeout` before the trial) | 0 | 2 |

The tall cylinder tipping after release is the main residual failure in
both modes; vision adds no perception failure (nothing missed or misnamed).

## Timing

Cycle times exclude the wait for the camera (reported separately as wait
per look). Throughput is lower in vision mode (2.9 vs 3.9 trials per minute)
because rendering the camera and running the model share the CPU with the
simulation; the vision run's real-time factor was 2.1 against 2.5.
