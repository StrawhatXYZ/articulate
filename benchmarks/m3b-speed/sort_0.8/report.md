# Benchmark report

- Date: 2026-10-09T11:57:42+00:00
- Commit: 30e8c94619beb69122376f4940db96305e1b4411 (dirty: true)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: ground_truth
- Seed: 42, trials per task: 20
- Complete: True
- Real-time factor: 3.10, trials per minute: 6.3

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| sort | 18/20 (90%) | 70–97% | 28.57 | 29.61 | 18 | Worst distance from bin centre (m) | 0.0111 | 0.0098 | 0.0171 | 20 | planning_failed: 2 | 0 |
