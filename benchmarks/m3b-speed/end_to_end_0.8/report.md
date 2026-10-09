# Benchmark report

- Date: 2026-10-09T13:52:42+00:00
- Commit: 15255d017280c2238a98428e472e2338e029dfc2 (dirty: true)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: vision
- Seed: 42, trials per task: 50
- Complete: True
- Real-time factor: 1.93, trials per minute: 3.7

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| instruct | 39/50 (78%) | 65–87% | 27.16 | 45.98 | 39 | Error (m) | n/a | n/a | n/a | 0 | execution: 3, infeasible: 4, outcome: 4 | 0 |
