# Benchmark report

- Date: 2026-10-09T11:54:03+00:00
- Commit: 30e8c94619beb69122376f4940db96305e1b4411 (dirty: true)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: ground_truth
- Seed: 42, trials per task: 20
- Complete: True
- Real-time factor: 2.88, trials per minute: 16.0

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 20/20 (100%) | 84–100% | 9.07 | 10.14 | 20 | Placement error (m) | 0.0060 | 0.0053 | 0.0128 | 20 | none | 1 |
