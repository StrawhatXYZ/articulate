# Benchmark report

- Date: 2026-10-05T07:58:06+00:00
- Commit: dca2d0cfb29c01234109b38ff915efe29eec654e (dirty: false)
- Architecture: x86_64
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 3.55, trials per minute: 19.1

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Error | Error mean | Error p95 | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|
| pick_place | 43/100 (43%) | 34–53% | 15.58 | 22.25 | Placement error (m) | 0.0318 | 0.0422 | dropped: 36, placement_error: 21 | 16 |
| reach | 100/100 (100%) | 96–100% | 3.48 | 4.91 | Tip error (m) | 0.0022 | 0.0045 | none | 0 |
