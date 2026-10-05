# Benchmark report

- Date: 2026-10-05T13:05:46+00:00
- Commit: 3538fa75f85c5ead39f6a474f1b2b663cce39fd9 (dirty: false)
- Architecture: x86_64
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 3.47, trials per minute: 17.4

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 83/100 (83%) | 74–89% | 15.61 | 21.63 | 83 | Placement error (m) | 0.0107 | 0.0073 | 0.0251 | 99 | dropped: 1, placement_error: 16 | 24 |
| reach | 100/100 (100%) | 96–100% | 3.48 | 4.92 | 100 | Tip error (m) | 0.0022 | 0.0020 | 0.0045 | 100 | none | 0 |
