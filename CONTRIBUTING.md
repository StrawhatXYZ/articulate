# Contributing

Thanks for your interest. This lab is built in small, measured milestones;
contributions that keep it reliable and reproducible are very welcome.

## Before you start

- For anything larger than a small fix, **open an issue first** to agree on
  the approach. Big changes get a short design note (see
  [`docs/design/`](docs/design/) for examples).
- Read [`AGENTS.md`](AGENTS.md): it explains what runs where, the
  conventions, the interfaces to keep stable and the known gotchas. It is
  written for coding agents and humans alike.

## Set up

You need Docker (Docker Desktop on macOS or Windows). Then:

```bash
./start-lab.sh     # build the image, start the container, build the workspace
./enter-lab.sh     # a shell in the container
```

The language features also need [Ollama](https://ollama.com) on the host with
`ollama pull gemma4:e2b`; the voice client has its own setup in the README.

## Make a change

1. **Write the test first** and watch it fail, then make it pass. Pure logic
   stays free of ROS so it can be tested with the fakes in
   `src/lab_simulation/test/fakes.py`.
2. **Run every check** inside the container:
   ```bash
   colcon build --symlink-install
   colcon test && colcon test-result --verbose    # linters + unit tests, all must pass
   ```
   and on the host, for the voice client: `python3 -m pytest -q host`.
3. **Behaviour changes need live evidence.**
   - Changes to the robot, controllers or simulation: launch it and run
     `ros2 run lab_simulation move_arm.py`. It must exit 0.
   - Grasping changes: run at least 20 pick-and-place trials
     (`TASK=pick_place TRIALS=20 ./run-benchmark.sh`).
   - Planner or prompt changes: run the planner benchmark
     (`ros2 run lab_simulation instruction_eval.py`) and report the
     **held-out** score. Never edit the 42 held-out cases in
     `instruction_cases.TEST`, and never tune a prompt on them.
4. **Lint shell scripts and the Dockerfile** as CI does: ShellCheck, and
   Hadolint at `--failure-threshold info`.

## Commits and pull requests

- [Conventional Commits](https://www.conventionalcommits.org/)
  (`feat(scope): ...`, `fix: ...`, `docs: ...`), one commit per logical
  change. Each commit builds and passes its own tests.
- Use an email address you are happy to make public (GitHub's
  `users.noreply.github.com` address works).
- In the pull request, say what changed, why, and how you checked it. Include
  benchmark numbers when behaviour changes.
- CI must be green before review.

## Licence

By contributing you agree that your contributions are licensed under the
[Apache License, Version 2.0](LICENSE), the project's licence.
