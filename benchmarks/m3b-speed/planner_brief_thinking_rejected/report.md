# Planner-only benchmark

Model `gemma4:e2b`, 2026-10-09T12:11:39. Scored on the safety check's dry run of each plan.

## dev

Success: 50/55 (91%, 95% CI 80-96%)

Infeasible (correct plan, no room on the table): 3; success on the rest: 50/52 (96%, 95% CI 87-99%)

Latency: median 2.4 s, p95 6.2 s, max 8.8 s

| Category | Success |
|---|---|
| box | 6/6 (100%, 95% CI 61-100%) |
| home | 2/3 (67%, 95% CI 21-94%) |
| language | 4/4 (100%, 95% CI 51-100%) |
| multi | 7/9 (78%, 95% CI 45-94%) |
| near | 5/6 (83%, 95% CI 44-97%) |
| question | 2/2 (100%, 95% CI 34-100%) |
| refusal | 8/8 (100%, 95% CI 68-100%) |
| region | 5/6 (83%, 95% CI 44-97%) |
| start | 9/9 (100%, 95% CI 70-100%) |
| swap | 2/2 (100%, 95% CI 34-100%) |

Outcomes: answered 2, done 40, refused 13

Failures:

- "put the green block beside the red cube" (fresh): outcome refused: step 1: no free spot left of red_cube
- "push the red cube over to the right side" (fresh): outcome refused: step 1: no free reachable spot for red_cube
- "put the red and the blue ones in their boxes" (fresh): green_block moved
- "first move the green block to the front, then put the red cube next to it" (fresh): outcome refused: step 2: no free spot left of green_block
- "go to sleep" (fresh): outcome refused: 

## test

Success: 36/42 (86%, 95% CI 72-93%)

Infeasible (correct plan, no room on the table): 3; success on the rest: 36/39 (92%, 95% CI 80-97%)

Latency: median 2.8 s, p95 5.2 s, max 8.8 s

| Category | Success |
|---|---|
| box | 7/7 (100%, 95% CI 65-100%) |
| home | 1/1 (100%, 95% CI 21-100%) |
| language | 4/4 (100%, 95% CI 51-100%) |
| multi | 4/5 (80%, 95% CI 38-96%) |
| near | 1/3 (33%, 95% CI 6-79%) |
| question | 3/4 (75%, 95% CI 30-95%) |
| refusal | 8/8 (100%, 95% CI 68-100%) |
| region | 2/4 (50%, 95% CI 15-85%) |
| start | 4/4 (100%, 95% CI 51-100%) |
| swap | 2/2 (100%, 95% CI 34-100%) |

Outcomes: answered 4, done 25, refused 13

Failures:

- "put the green block on the left side of the blue cylinder" (fresh): outcome refused: step 1: no free spot left of blue_cylinder
- "set the red cube down next to the cylinder" (blue_moved): outcome refused: step 1: no free spot right of blue_cylinder
- "move the blue cylinder to the left of the table" (fresh): outcome refused: missing target
- "slide the red cube towards the back" (fresh): outcome refused: missing target
- "move the red cube to the right, and put the blue cylinder next to it on its left" (fresh): outcome refused: step 1: no free reachable spot for red_cube
- "how many pieces are on the table?" (red_in_bin): outcome answered: There are three pieces on the table: the blue_cylinder, the green_block, and the red_cube.
