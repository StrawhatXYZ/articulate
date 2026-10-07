# Benchmark report

- Date: 2026-10-07T08:28:47+00:00
- Commit: 6526dbd3b8c64e85f06b32dbaaac85d3d27f5779 (dirty: false)
- Architecture: aarch64
- Robot: so101, grasp mode: latched
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 3.69, trials per minute: 26.6

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 100/100 (100%) | 96–100% | 12.12 | 13.14 | 100 | Placement error (m) | 0.0056 | 0.0052 | 0.0122 | 100 | none | 0 |
| reach | 100/100 (100%) | 96–100% | 1.27 | 1.45 | 100 | Tip error (m) | 0.0004 | 0.0004 | 0.0007 | 100 | none | 0 |
