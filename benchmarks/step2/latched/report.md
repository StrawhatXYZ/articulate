# Benchmark report

- Date: 2026-10-07T15:31:57+00:00
- Commit: fd3d5e8edef85e3c9fd841149e4e3620d4a87e8f (dirty: false)
- Architecture: aarch64
- Robot: so101, grasp mode: latched
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 3.32, trials per minute: 10.5

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 98/100 (98%) | 93–99% | 11.47 | 12.17 | 98 | Placement error (m) | 0.0066 | 0.0052 | 0.0143 | 100 | placement_error: 2 | 0 |
| reach | 100/100 (100%) | 96–100% | 1.30 | 1.48 | 100 | Tip error (m) | 0.0004 | 0.0004 | 0.0008 | 100 | none | 0 |
| sort | 93/100 (93%) | 86–97% | 37.57 | 39.73 | 93 | Worst distance from bin centre (m) | 0.0120 | 0.0103 | 0.0183 | 100 | placement_error: 1, timeout: 3, unreachable: 3 | 0 |
