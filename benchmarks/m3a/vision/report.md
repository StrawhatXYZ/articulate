# Benchmark report

- Date: 2026-10-08T08:17:15+00:00
- Commit: 59edf4b76b2caa78c025571360b37a83e71af509 (dirty: false)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: vision
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 2.07, trials per minute: 2.9

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| sort | 92/100 (92%) | 85–96% | 34.95 | 36.18 | 92 | Worst distance from bin centre (m) | 0.0134 | 0.0107 | 0.0297 | 98 | placement_error: 5, planning_failed: 1, timeout: 2 | 0 |

Detection quality (scored against the true piece positions):

| Task | Observations | Named correctly | Missed | False | Position error mean / p95 (mm) | Latency mean (ms) |
|---|---|---|---|---|---|---|
| sort | 294 | 588 (100%) | 0 | 0 | 1.4 / 1.8 | 1222 |
