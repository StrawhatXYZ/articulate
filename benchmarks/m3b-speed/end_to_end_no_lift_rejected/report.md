# Benchmark report

- Date: 2026-10-09T13:15:05+00:00
- Commit: 95c8d05f36fd239ec28afd851d5537cc6a48dbbd (dirty: true)
- Architecture: aarch64
- Robot: so101, grasp mode: latched, perception: vision
- Seed: 42, trials per task: 50
- Complete: True
- Real-time factor: 1.94, trials per minute: 4.0

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| instruct | 32/50 (64%) | 50–76% | 23.39 | 40.50 | 32 | Error (m) | n/a | n/a | n/a | 0 | check: 1, execution: 4, infeasible: 4, outcome: 8, setup: 1 | 0 |
