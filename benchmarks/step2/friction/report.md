# Benchmark report

- Date: 2026-10-07T08:34:42+00:00
- Commit: 6526dbd3b8c64e85f06b32dbaaac85d3d27f5779 (dirty: false)
- Architecture: aarch64
- Robot: so101, grasp mode: friction
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 3.63, trials per minute: 18.3

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 64/100 (64%) | 54–73% | 11.83 | 12.72 | 64 | Placement error (m) | 0.0067 | 0.0060 | 0.0133 | 64 | dropped: 36 | 7 |
