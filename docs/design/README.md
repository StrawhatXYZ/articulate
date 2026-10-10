# Design records

Each milestone was designed before it was built. Two documents record it:

- **`specs/`**: the design. What the milestone is for, the decisions taken
  (with the alternatives rejected and why), error handling, how it is
  measured and what counts as done.
- **`plans/`**: the implementation plan derived from the spec. It lists tasks
  in order, each test-first, with the files touched and the checks that
  close the task.

They are kept as written at the time, so they show how the project got here.
Later milestones may have changed some details; the README and the code are
the current reference.

| Milestone | Spec | Plan | Results |
|---|---|---|---|
| Step 1: benchmark and native arm64 image | [spec](specs/2026-10-05-step1-benchmark-native-image-design.md) | [plan](plans/2026-10-05-step1-benchmark-native-image.md) | [`benchmarks/baseline/`](../../benchmarks/baseline/) |
| Step 2: SO-101 with MoveIt and two-finger grasping | [spec](specs/2026-10-07-step2-so101-moveit-design.md) | [plan](plans/2026-10-07-step2-so101-moveit.md) | [`benchmarks/step2/`](../../benchmarks/step2/) |
| M3a: sorting from an overhead camera | [spec](specs/2026-10-07-m3a-vision-design.md) | [plan](plans/2026-10-07-m3a-vision.md) | [`benchmarks/m3a/`](../../benchmarks/m3a/) |
| M3b: instructions to robot actions | [spec](specs/2026-10-09-m3b-language-design.md) | [plan](plans/2026-10-09-m3b-language.md) | [`benchmarks/m3b/`](../../benchmarks/m3b/), [`benchmarks/m3b-speed/`](../../benchmarks/m3b-speed/) |
