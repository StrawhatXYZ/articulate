# M3b: Language — Instructions to Robot Actions

- **Status:** Draft, awaiting review
- **Date:** 9 October 2026
- **Roadmap position:** Step 3, second of three AI milestones (M3a vision, M3b language, M4 learning)
- **Builds on:** M3a (camera perception) and PR #3 by @MunuriGangadhar (Gemma planner demo)

## 1. Purpose

Today the robot sorts on a fixed script. M3b lets a person type or say an
instruction ("put the red cube back where it was", "swap the cube and the
cylinder", "what's in the boxes?") and have the arm carry it out in the
simulation, or refuse it with a reason. A local language model plans; a
deterministic safety check decides what may move.

Success means:

1. A typed or spoken instruction on the host (macOS or Windows) is planned by
   Gemma 4 E2B and executed by the arm, with progress and a result reported.
2. No unsafe, impossible or malformed request ever moves the arm.
3. Planner accuracy and end-to-end success are measured and committed as
   baselines.

Out of scope: wake-word or hands-free listening, cloud models, new piece types
or tasks, learned policies (M4), detecting pieces inside the boxes.

## 2. Relationship to PR #3

PR #3 built a working demo of this idea. Each of its choices was reviewed; the
outcome:

| PR #3 choice | Decision |
|---|---|
| Gemma 4 E2B via Ollama on the host | Kept (confirmed by the spike, section 4) |
| Native tool calling; refusal = no tool called | Tool calling kept; explicit `refuse` and `answer` tools added |
| Tool set (move to box / start / near / region, swap, sort all, home) | Kept |
| `targets.py` (safe, reachable target spots) | Kept |
| Re-picking from a box (`_approach_yaws`) | Kept, verified by pick_place trials |
| Planner on the host; JSON plans on `/lab/plan`, scene on `/lab/scene` | Planner moves into the container; one ROS action replaces the topics |
| Keyword / EmbeddingGemma sentence matching (`/lab/instruction`) | Removed (the planner does this) |
| EmbeddingGemma 2 perception backend | Removed (SigLIP 2 measured 588/588; EmbeddingGemma is gated and slower) |
| Ground-truth poses for pieces in boxes | Replaced by placement memory |
| Two-sided region definitions (`brain.region_of`, `targets.REGIONS`) | One definition in `layout` |
| faster-whisper with CUDA (Windows) | Automatic engine: mlx-whisper on Apple Silicon, faster-whisper elsewhere |
| Demo arm speed 1.0, acceleration tied to speed | Demo default 0.5; acceleration unchanged |
| `torchvision`, `sentence-transformers` in the Dockerfile | Removed |
| AGENTS.md (Windows, hand-installed venv) | Kept, rewritten for macOS and Windows |

The contributor is credited in the commits that carry their code.

## 3. Decisions

| Topic | Decision | Rejected alternative |
|---|---|---|
| Model | Gemma 4 E2B (`gemma4:e2b`, 4.6 GB) via Ollama on the host GPU | E4B (6.6 GB, does not fit beside the 8 GB Docker VM on a 16 GB Mac); cloud models |
| Output format | Native tool calls, thinking on, `refuse(reason)` and `answer(text)` tools | JSON-schema-forced output (23/35, 26/42 in the spike); thinking off (29/35, 28/42) |
| Runaway thinking | `num_predict` 1024, 30 s timeout, one retry | No cap (two replies of 572 s and 903 s in the spike) |
| Where the planner runs | In the container, calling Ollama at `host.docker.internal:11434` | On the host (benchmark could not run end to end; logic outside CI) |
| Interface | ROS action `/lab/instruct` (new package `lab_interfaces`) | JSON on `std_msgs/String` topics |
| Pieces in boxes | Placement memory; camera detections always win | Ground truth (breaks M3a's rule); unmasking the boxes (occluded, new perception work) |
| Speech | Push-to-talk; mlx-whisper on Apple Silicon, faster-whisper elsewhere | One engine for all (mlx is Mac-only; faster-whisper is CPU-only on a Mac) |

## 4. Spike results (recorded 9 October 2026)

Gemma 4 E2B on an M4 (16 GB), Ollama 0.33.1, temperature 0. 35 dev cases (PR
#3's phrasings, which its prompt was tuned on) and 42 held-out cases written
before any run, scored on symbolic final piece locations.

| Setup | Dev | Held-out | Median |
|---|---|---|---|
| PR #3 planner as-is | 34/35 | 35/42 | 5.5 s |
| JSON-schema-forced, thinking off | 23/35 | 26/42 | 3.4 s |
| JSON-schema-forced, thinking on (stopped) | 6/10 | – | up to 116 s |
| **PR tools + `refuse`/`answer`, thinking on** | **34/35** | **42/42** | **5.3 s** |
| Same, thinking off | 29/35 | 28/42 | 0.7 s |

- Gemma 4 E2B thinks by default in Ollama; without thinking it writes tool
  calls as plain text and accuracy collapses.
- Normal replies use 180–380 tokens; two of about 230 replies ran away (572 s,
  903 s), hence the cap.
- The PR planner alone fails all four questions and four of eight refusals
  (it sends the arm home or uses the wrong box instead).
- Remaining miss: "first move the green block to the front, then put the red
  cube next to it" (loses the referent of "it").
- Model memory under simulation load was not measured in the spike; the
  benchmark records it.

## 5. Design

### 5.1 Architecture

```
host                                     container
voice client (push-to-talk, Whisper)
  text ──── /lab/instruct (action) ────▶ instruction node
                                          1. scene: camera detections + placement memory
Ollama (Gemma 4 E2B) ◀── HTTP ───────────  2. planner: sentence + scene -> tool calls
                                          3. safety check: whole plan, before moving
                                          4. execute: targets -> PickPlaceTask -> MoveIt
  feedback / result ◀────────────────────  5. update placement memory
```

| Unit | Location | Job |
|---|---|---|
| `planner.py` | `lab_simulation`, pure Python | Sentence + scene text -> tool calls, `refuse` or `answer`; Ollama client with cap, timeout and retry; parses native and text-written calls |
| `plan_check.py` | `lab_simulation`, pure Python | Tool calls -> validated steps, or a refusal reason; never raises |
| `targets.py` | `lab_simulation`, pure Python | Target -> safe, reachable table spot (from PR #3) |
| `memory.py` | `lab_simulation`, pure Python | Placement memory and start positions |
| `instruction_node.py` | `lab_simulation/scripts` | Serves `/lab/instruct`, one goal at a time; runs steps; owns the memory |
| `Instruct.action` | new package `lab_interfaces` | Request `text`; feedback `stage`, `message`; result `outcome`, `message`, `steps` |
| `host/voice.py` | host | Microphone -> text -> action goal; prints feedback |

The benchmark uses the same planner, check and executor as the action.

### 5.2 Tools offered to the model

`move_piece(piece, target, other_piece, side, region)` with target `bin`,
`start`, `near` or `region`; `swap_pieces(piece_a, piece_b)`; `sort_all()`;
`go_home()`; `refuse(reason)`; `answer(text)`. The prompt is PR #3's with the
refusal rule replaced by `refuse` and an `answer` rule added (spike setup 4).
The model never supplies coordinates.

The scene text comes only from camera detections and memory, e.g.
`red_cube (red cube): in its box (placed by me)`,
`blue_cylinder (blue cylinder): on the table, front area; moved from its start (right area)`,
`green_block (green block): not visible`. Table areas (left, right, front,
back, center) are defined once in `layout`.

### 5.3 Safety check

Any failure refuses the whole plan before anything moves.

1. **Shape.** Tool names from the fixed set; arguments are a dict of strings;
   missing required fields, wrong types, nested values refuse. `refuse` or
   `answer` must be the only call.
2. **Meaning.** A piece is its exact name or a single unambiguous colour or
   shape word; sides and regions from the fixed lists; at most 8 steps after
   expansion (a swap is 3 moves).
3. **Dry run.** Step by step over the current scene: every piece involved is
   known (seen or remembered); every target resolves through `targets.py`;
   `near` needs the other piece on the table; a swap needs both pieces on the
   table.

During execution each step re-observes the table; if a step is no longer
valid the plan stops, the arm goes home, and the result names the step.

### 5.4 Placement memory

| State | Set when | Cleared when |
|---|---|---|
| on the table (position) | the camera detects it | it is picked up |
| in its box | a placement into its box completes (grasp verified, release confirmed) | a pick from the box is verified, or the camera sees it on the table |
| unknown | neither | – |

Start position = where the camera first saw the piece after the last scene
reset. Resets clear memory and starts. A failed placement records nothing.
In vision mode nothing the planner or check uses comes from ground truth.

### 5.5 Outcomes

`done`, `answered`, `refused` (model or check), `not_understood` (no usable
call after one retry), `failed` (execution; names the step), `planner_unavailable`
(Ollama unreachable or model missing), `cancelled`. A goal sent while busy is
rejected. Feedback stages: heard, planning, plan, step i/N, done.

## 6. Error handling

- Planner: 1024-token cap, 30 s timeout, one retry, then `not_understood`.
  Ollama down: `planner_unavailable` at once; no keyword fallback. The node
  starts and warms the model up, but serves even if Ollama is down.
- Failure mid-plan: unlatch, open the jaws partly, home with pieces and boxes
  out of the planning scene, report the step. A carried piece drops; memory is
  unchanged and the next observation finds it.
- Cancel takes effect between steps, then the arm goes home.
- Unexpected exceptions are caught at the action boundary: `failed: internal
  error`, logged with a traceback; the node keeps serving.
- Voice client: empty transcript -> "nothing heard", nothing sent; container
  unreachable -> a clear message.
- Each request appends one JSON line to a log: transcript, scene text, raw
  tool calls, check verdict, step results, timings.

## 7. Measurement

**Planner-only** (Ollama, no simulation; about 100 cases, about 10 min). The 42
held-out cases are frozen and never used for tuning; dev grows to about 58.
Reported per category (into box, start, near, region, multi-step, swap, other
languages, questions, refusals): accuracy with Wilson intervals, median and
p95 latency, retries, `not_understood`, model memory.

**End-to-end** (simulation, vision mode): 25 instructions x 2 seeded scenes =
50 runs, about 1.5 h. Scenes randomise positions, yaws and light as in M3a;
some start with pieces in boxes. Scored on ground-truth final positions:

| Expected | Pass if |
|---|---|
| in its box | inside that box |
| back to start | within 3 cm of its start |
| next to a piece | within 4 cm of the nearest valid spot beside it, on the requested side if given |
| in a table area | inside that area |
| not mentioned | moved less than 1 cm |
| refusal | nothing moved |

Failures are attributed to planner, check or execution (with stage).

**Grasping regression:** at least 20 pick_place trials at benchmark speed 0.5,
compared with the step 2 baseline.

Results go to `benchmarks/m3b/` with `comparison.md`.

## 8. Testing

All in `colcon test`, no Ollama required:

- `plan_check`: every rule, plus a seeded random test of thousands of malformed
  inputs (never raises; refuses or yields a valid plan).
- `planner`: a fake Ollama HTTP server with scripted replies (native and
  text-written calls, mixed `refuse`, cap sent, timeout -> retry ->
  `not_understood`, server down -> `planner_unavailable`).
- `memory`: every transition, including camera-wins and reset.
- Executor and node logic with fakes: order, stop on failure, cancel between
  steps, busy rejection, exception -> `failed` with the node alive.
- `targets` and `_approach_yaws`: PR #3's tests plus approach order.
- Voice: engine choice and text handling with a fake engine; a real-model test
  skipped without weights.
- `lab_interfaces`: builds; action round trip.

## 9. Success criteria

1. Typing or saying an instruction on the Mac moves the arm in the simulation,
   with progress and result printed.
2. Planner held-out accuracy at least 90%; no unsafe or impossible request is
   executed (guaranteed by the check and its random-input test).
3. A 50-run end-to-end baseline with its interval and failure breakdown is
   committed (no pass mark; it is the reference for later work).
4. pick_place success within the step 2 baseline's interval.
5. All tests pass and CI is green; README, AGENTS.md (macOS and Windows) and
   setup are documented.
