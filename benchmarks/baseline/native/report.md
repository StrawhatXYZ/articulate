# Benchmark report

- Date: 2026-10-05T17:46:47+00:00
- Commit: 3538fa75f85c5ead39f6a474f1b2b663cce39fd9 (dirty: false)
- Architecture: aarch64
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 4.44, trials per minute: 22.2

Cycle times cover successful trials only; errors cover trials where the final error was measured (n given for each).

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) | Error | Error mean | Error median | Error p95 | Error n (measured) | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pick_place | 84/100 (84%) | 76–90% | 15.56 | 21.63 | 84 | Placement error (m) | 0.0105 | 0.0071 | 0.0249 | 99 | dropped: 1, placement_error: 15 | 24 |
| reach | 100/100 (100%) | 96–100% | 3.48 | 4.91 | 100 | Tip error (m) | 0.0024 | 0.0020 | 0.0047 | 100 | none | 0 |
