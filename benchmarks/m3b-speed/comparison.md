# M3b speed-up: what we tried, and what we kept

Goal: shorten a spoken instruction (about 26 s at the M3b baseline) without
losing reliability. Every change was checked before it was kept.

## Where the time went (one instruction, measured live)

| Part | Time |
|---|---|
| Speech to text | 0.5 s |
| Language model | 4-6 s: reading the prompt 0.1 s (cached), writing ~200 tokens at ~50 tokens/s, three quarters of them hidden thinking |
| Arm motion | 12-13.5 s: 8 planned moves, 2 camera looks, gripper opening and closing, settle checks |

## Results

| Change | Saves | Check | Kept? |
|---|---|---|---|
| Reuse the camera look the plan was made from for the first step | ~1 s | unit tests; end to end 39/50 | **Kept** |
| Arm speed 0.8 instead of 0.5 (demo and instructions only) | ~1.5 s per move (10.6-11.6 s vs 12-13.5 s) | end to end 39/50 ([`end_to_end_0.8`](end_to_end_0.8)) vs 38/50 | **Kept** |
| Carry straight from the table, without lifting first | ~1 s | end to end fell to 32/50: the held piece swept into other pieces in 5 trials ([`end_to_end_no_lift_rejected`](end_to_end_no_lift_rejected)) | Rejected |
| Ask the model to "think briefly" | ~3 s (planning 6.4 s → 2.8 s) | dev set 50/55 (better), but held-out fell from 40/42 to 36/42: missing targets, a wrong count ([`planner_brief_thinking_rejected`](planner_brief_thinking_rejected)) | Rejected |
| Free memory on the Mac | planning back from 6-9 s to ~4 s | macOS was swapping 13-18 GB | Advice: restart Docker Desktop before a demo, close spare tabs |

Net: about **26 s → 23.5 s** per instruction (about 10% faster), with end-to-end
success unchanged (39/50 against 38/50). The pick and sort benchmarks keep
speed 0.5 so their numbers stay comparable with earlier baselines.

## What would make it faster next

- A faster planner: a bigger GPU budget or a model that needs less thinking,
  measured on the frozen held-out set (the "think briefly" prompt shows the
  speed is there, but this 2B model needs its thinking to stay accurate).
- Smoother motion: a single trajectory through several waypoints instead of
  eight separate start-stop moves, keeping the lift clear of other pieces.
