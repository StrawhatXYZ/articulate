# Benchmark report

- Date: 2026-10-09T09:10:50+00:00
- Commit: 571173dc4875874a67b1a2a62adcdd58e9d1117c (dirty: true)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: ground_truth
- Seed: 42, trials per task: 20
- Complete: True
- Real-time factor: 2.50, trials per minute: 4.0

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| sort | 17/20 (85%) | 64–95% | 35.99 | 37.65 | 17 | Worst distance from bin centre (m) | 0.0120 | 0.0096 | 0.0220 | 20 | placement_error: 1, planning_failed: 1, timeout: 1 | 0 |
