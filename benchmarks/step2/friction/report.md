# Benchmark report

- Date: 2026-10-07T14:43:17+00:00
- Commit: 9434766db0ac8c14da5dedaa77fc143e2a03f8fb (dirty: false)
- Architecture: aarch64
- Robot: so101, grasp mode: friction
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 3.24, trials per minute: 16.8

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 73/100 (73%) | 64–81% | 11.15 | 11.90 | 73 | Placement error (m) | 0.0065 | 0.0057 | 0.0123 | 75 | dropped: 25, placement_error: 2 | 7 |
