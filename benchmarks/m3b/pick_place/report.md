# Benchmark report

- Date: 2026-10-09T09:05:14+00:00
- Commit: 571173dc4875874a67b1a2a62adcdd58e9d1117c (dirty: true)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: ground_truth
- Seed: 42, trials per task: 20
- Complete: True
- Real-time factor: 2.34, trials per minute: 10.8

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 20/20 (100%) | 84–100% | 11.29 | 12.43 | 20 | Placement error (m) | 0.0065 | 0.0055 | 0.0167 | 20 | none | 0 |
