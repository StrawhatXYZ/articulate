# Benchmark report

- Date: 2026-10-08T05:23:04+00:00
- Commit: bf8b72af28f674f067bebfd76dd5d3e0ecdb83f6 (dirty: false)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: ground_truth
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 2.51, trials per minute: 3.9

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| sort | 93/100 (93%) | 86–97% | 37.19 | 38.87 | 93 | Worst distance from bin centre (m) | 0.0104 | 0.0088 | 0.0198 | 100 | placement_error: 2, timeout: 5 | 0 |
