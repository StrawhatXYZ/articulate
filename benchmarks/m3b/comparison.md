# M3b baseline: instructions to robot actions

Gemma 4 E2B (`gemma4:e2b`, Ollama 0.33.1, temperature 0, thinking on,
1024-token cap) on an Apple M4 (16 GB) with the simulation running; native
arm64 container. Code at commit `05afb16` (the end-to-end run's tree also
held uncommitted README edits).

## Planner only ([`planner`](planner))

Every instruction is planned by the model and checked by the safety check;
the check's dry run of the plan is scored on where the pieces end up. The
42 held-out (TEST) cases were written before any model was run and are
frozen; DEV holds PR #3's phrasings and later additions.

| Set | Success | 95% interval | Infeasible | Success on feasible | Median / p95 time |
|---|---|---|---|---|---|
| Held-out (TEST) | 40/42 (95%) | 84–99% | 2 | **40/40** | 6.4 / 12.1 s |
| Dev | 49/55 (89%) | 78–95% | 4 | 49/51 | 5.7 / 10.6 s |

*Infeasible*: the model's plan was right, but the crowded table has no room
for it (the free strip right of the boxes holds about one piece per third),
so the safety check refused it, for example "no free spot left of
blue_cylinder". Both held-out misses are of this kind. Of the
dev misses, two are the model's: "put the red and the blue ones in their
boxes" became sort-everything, and "go to sleep" was refused instead of
going home.

Held-out by category: into the box 7/7, back to start 4/4, next to 2/3,
area 4/4, several steps 4/5, swap 2/2, home 1/1, other languages 4/4,
questions 4/4, refusals 8/8.

## End to end ([`end_to_end`](end_to_end))

25 instructions × 2 random scenes (positions, angles, light), vision mode:
the robot plans only from the camera and its memory of what it put in the
boxes; ground truth only scores where the pieces end up.

| | Result |
|---|---|
| Success | **38/50 (76%)**, 95% interval 63–86% |
| Success on feasible trials | 38/46 (83%), 69–91% |
| Planning time per instruction, median | 8.4 s |
| Instruction time, median (planning + motion) | 26 s |

| Category | Success |
|---|---|
| Into its box | 6/6 |
| Several pieces | 6/8 |
| Back to start (out of the box) | 2/6 |
| Next to a piece | 2/6 |
| Area of the table | 5/6 |
| Swap | 1/2 |
| Other languages | 4/4 |
| Questions | 4/4 |
| Refusals (unsafe, non-existent) | 6/6 |
| Home | 2/2 |

Failures by cause (12):

- **Infeasible (4):** no room on the side or area asked for, or no free
  spot to swap through. Refused honestly; nothing moved.
- **Model (4):** "put the red and the blue ones in their boxes" planned as
  sort-everything (twice; once the extra move also failed); "put the green
  block in front of the red cube" planned as "the front of the table"
  (twice).
- **Picking out of a box (4):** two missed grasps and one piece placed off
  its start when picking from the remembered pose, and one "not visible"
  after the setup. Picking from memory is the weakest link: the estimated
  pose (release pose plus nominal grip) is not yet accurate enough for the
  partly opened jaws inside a box.

No unsafe or non-existent request moved anything (6/6), and no plan moved
a piece the instruction did not mention except the model's two
sort-everything plans.

## Grasping regression ([`pick_place`](pick_place), [`sort`](sort))

The groundwork commit changed the pick skill (opening order, picking from a
box, recording the placed pose). Checked at benchmark speed 0.5, seed 42,
20 trials each:

| Task | M3b | Step 2 / M3a baseline |
|---|---|---|
| pick_place | 20/20 (84–100%) | 98% |
| sort | 17/20 (64–95%) | 93% (86–97%) |

Both within the earlier baselines' intervals. Two of the three sort
failures were timeouts while tests ran on the same CPU.

## What to improve next

1. Picking from a box: re-estimate the piece from the depth image around the
   box, or open the jaws wider on the first approach.
2. The model's "the red and the blue ones" and "in front of the cube":
   add these patterns to the prompt's rules, measured on DEV only.
3. A roomier table or smaller pieces would make "next to" and "swap"
   feasible more often; the geometry, not the model, limits them now.
4. Each request stands alone: "put it back" needs the piece named.
