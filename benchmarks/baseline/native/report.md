# Benchmark report

- Date: 2026-10-05T08:08:45+00:00
- Commit: dca2d0cfb29c01234109b38ff915efe29eec654e (dirty: false)
- Architecture: aarch64
- Seed: 42, trials per task: 100
- Complete: True
- Real-time factor: 4.93, trials per minute: 26.6

| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Error | Error mean | Error p95 | Failures | Disturbed |
|---|---|---|---|---|---|---|---|---|---|
| pick_place | 42/100 (42%) | 33–52% | 15.56 | 22.29 | Placement error (m) | 0.0189 | 0.0415 | dropped: 37, placement_error: 21 | 15 |
| reach | 100/100 (100%) | 96–100% | 3.48 | 4.92 | Tip error (m) | 0.0023 | 0.0045 | none | 0 |
