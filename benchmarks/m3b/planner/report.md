# Planner-only benchmark

Model `gemma4:e2b`, 2026-10-09T10:26:52. Scored on the safety check's dry run of each plan.

## dev

Success: 49/55 (89%, 95% CI 78-95%)

Infeasible (correct plan, no room on the table): 4; success on the rest: 49/51 (96%, 95% CI 87-99%)

Latency: median 5.7 s, p95 10.6 s, max 16.9 s

| Category | Success |
|---|---|
| box | 6/6 (100%, 95% CI 61-100%) |
| home | 2/3 (67%, 95% CI 21-94%) |
| language | 4/4 (100%, 95% CI 51-100%) |
| multi | 7/9 (78%, 95% CI 45-94%) |
| near | 4/6 (67%, 95% CI 30-90%) |
| question | 2/2 (100%, 95% CI 34-100%) |
| refusal | 8/8 (100%, 95% CI 68-100%) |
| region | 5/6 (83%, 95% CI 44-97%) |
| start | 9/9 (100%, 95% CI 70-100%) |
| swap | 2/2 (100%, 95% CI 34-100%) |

Outcomes: answered 2, done 39, refused 14

Failures:

- "put the green block beside the red cube" (fresh): outcome refused: step 1: no free spot right of red_cube
- "push the red cube over to the right side" (fresh): outcome refused: step 1: no free reachable spot for red_cube
- "put the red and the blue ones in their boxes" (fresh): green_block moved
- "first move the green block to the front, then put the red cube next to it" (fresh): outcome refused: step 2: no free spot right of green_block
- "take the green block and put it next to the cylinder" (fresh): outcome refused: step 1: no free spot left of blue_cylinder
- "go to sleep" (fresh): outcome refused: I cannot go to sleep.

## test

Success: 40/42 (95%, 95% CI 84-99%)

Infeasible (correct plan, no room on the table): 2; success on the rest: 40/40 (100%, 95% CI 91-100%)

Latency: median 6.4 s, p95 12.1 s, max 24.9 s

| Category | Success |
|---|---|
| box | 7/7 (100%, 95% CI 65-100%) |
| home | 1/1 (100%, 95% CI 21-100%) |
| language | 4/4 (100%, 95% CI 51-100%) |
| multi | 4/5 (80%, 95% CI 38-96%) |
| near | 2/3 (67%, 95% CI 21-94%) |
| question | 4/4 (100%, 95% CI 51-100%) |
| refusal | 8/8 (100%, 95% CI 68-100%) |
| region | 4/4 (100%, 95% CI 51-100%) |
| start | 4/4 (100%, 95% CI 51-100%) |
| swap | 2/2 (100%, 95% CI 34-100%) |

Outcomes: answered 4, done 28, refused 10

Failures:

- "put the green block on the left side of the blue cylinder" (fresh): outcome refused: step 1: no free spot left of blue_cylinder
- "move the red cube to the right, and put the blue cylinder next to it on its left" (fresh): outcome refused: step 1: no free reachable spot for red_cube
