# Benchmark report

- Date: 2026-10-09T10:56:33+00:00
- Commit: 05afb16186861273e85592ea165ffd6c84b71952 (dirty: true)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: vision
- Seed: 42, trials per task: 50
- Complete: True
- Real-time factor: 1.39, trials per minute: 2.7

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| instruct | 38/50 (76%) | 63–86% | 27.05 | 49.77 | 38 | Error (m) | n/a | n/a | n/a | 0 | execution: 4, infeasible: 4, outcome: 4 | 0 |
