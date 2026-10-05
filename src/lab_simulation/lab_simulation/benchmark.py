"""
Benchmark command line: run seeded trials, write reports, compare runs.

Usage (inside the container, with the simulation running):
  ros2 run lab_simulation benchmark.py run --task all --trials 100 --seed 42
  ros2 run lab_simulation benchmark.py compare results/A results/B

`run` is normally started by launch/benchmark.launch.py. ROS is imported only
by `run`, so the rest of this module is testable without a simulation.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
import time
from typing import Callable, List, Sequence, Tuple

from . import metrics
from .tasks import SimulationUnresponsive, TrialResult

EXIT_OK = 0
EXIT_NO_SIMULATION = 2
EXIT_UNRESPONSIVE = 3
UNTHROTTLED_RTF = 1000


def make_run_dir(base: Path, now: datetime) -> Path:
    """Create and return a new, uniquely named run directory under base."""
    stamp = now.strftime('%Y%m%d-%H%M%S')
    candidate = base / stamp
    suffix = 1
    while candidate.exists():
        candidate = base / f'{stamp}-{suffix}'
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def execute(tasks: Sequence, trials_per_task: int, seed: int,
            reset: Callable[[random.Random], bool], trials_path: Path) -> Tuple[List[dict], bool]:
    """
    Run every task trials_per_task times, appending each trial to trials_path.

    Each task gets its own random stream, derived from the seed and the task
    name, so running one task alone reproduces the same trials as running
    all. Returns the trial dicts and whether the run completed.
    """
    trials: List[dict] = []
    with open(trials_path, 'a') as out:
        for task in tasks:
            rng = random.Random(f'{seed}-{task.name}')
            for index in range(trials_per_task):
                try:
                    if reset(rng):
                        result = task.run(rng, index)
                    else:
                        result = TrialResult(task.name, index, False, 'timeout', 0.0,
                                             details={'stage': 'reset'})
                except SimulationUnresponsive as error:
                    print(f'aborting: {error}', file=sys.stderr)
                    return trials, False
                record = result.to_dict()
                trials.append(record)
                out.write(json.dumps(record) + '\n')
                out.flush()
                print(f'{task.name} {index + 1}/{trials_per_task}: '
                      f"{'ok' if record['success'] else record['failure']}", flush=True)
    return trials, True


def provenance() -> dict:
    """Return where and how this run was produced."""
    try:
        gazebo = subprocess.run(['gz', 'sim', '--versions'], capture_output=True, text=True,
                                timeout=10).stdout.strip() or 'unknown'
    except (OSError, subprocess.SubprocessError):
        gazebo = 'unknown'
    return {
        'git_commit': os.environ.get('LAB_GIT_COMMIT', 'unknown'),
        'git_dirty': os.environ.get('LAB_GIT_DIRTY', 'unknown'),
        'arch': platform.machine(),
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'gazebo': gazebo,
        'date': datetime.now(timezone.utc).isoformat(timespec='seconds'),
    }


def write_outputs(run_dir: Path, trials: List[dict], run_info: dict) -> dict:
    """Write summary.json and report.md to run_dir and return the summary."""
    summary = metrics.summarize(trials, run_info)
    (run_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (run_dir / 'report.md').write_text(metrics.render_report(summary))
    return summary


def compare_dirs(a: Path, b: Path) -> str:
    """Return the Markdown comparison of two run directories."""
    summary_a = json.loads((a / 'summary.json').read_text())
    summary_b = json.loads((b / 'summary.json').read_text())
    return metrics.render_comparison(metrics.compare(summary_a, summary_b), a.name, b.name)


def set_real_time_factor(value: float) -> bool:
    """Set the Gazebo real-time factor through the set_physics service."""
    command = ['gz', 'service', '-s', '/world/table_world/set_physics',
               '--reqtype', 'gz.msgs.Physics', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '5000', '--req', f'max_step_size: 0.001, real_time_factor: {value}']
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return False
    return 'true' in result.stdout


def run(args) -> int:
    """Run the benchmark against the live simulation."""
    import rclpy
    from rclpy.parameter import Parameter

    from .arm import Arm
    from .layout import sample_layout
    from .ros_util import sim_now, spin_until
    from .scene import Scene
    from .tasks import HOME, MOVE_TIMEOUT_S, PickPlaceTask, ReachTask

    rclpy.init()
    node = rclpy.create_node(
        'benchmark', parameter_overrides=[Parameter('use_sim_time', value=True)])
    arm = Arm(node)
    scene = Scene(node)
    try:
        ready = spin_until(
            node, lambda: sim_now(node) > 0 and arm.joint_positions() is not None
            and scene.has_poses(), sim_timeout_s=float('inf'), wall_timeout_s=60.0)
    except SimulationUnresponsive:
        ready = False
    if not ready:
        print('simulation not running: no clock, joint states or piece poses', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return EXIT_NO_SIMULATION

    rtf_set = set_real_time_factor(UNTHROTTLED_RTF)
    now = (lambda: sim_now(node))
    all_tasks = {'reach': ReachTask(arm, now), 'pick_place': PickPlaceTask(arm, scene, now)}
    tasks = list(all_tasks.values()) if args.task == 'all' else [all_tasks[args.task]]

    def reset(rng: random.Random) -> bool:
        scene.detach_all()
        if not arm.move_to(HOME, MOVE_TIMEOUT_S).reached:
            return False
        return scene.reset(sample_layout(rng))

    run_dir = make_run_dir(Path(args.out), datetime.now())
    wall_start, sim_start = time.monotonic(), sim_now(node)
    trials, complete = execute(tasks, args.trials, args.seed, reset, run_dir / 'trials.jsonl')
    wall, sim = time.monotonic() - wall_start, sim_now(node) - sim_start
    run_info = {
        'tasks': [task.name for task in tasks],
        'trials_per_task': args.trials,
        'seed': args.seed,
        'complete': complete,
        'unthrottled': rtf_set,
        'wall_time_s': wall,
        'sim_time_s': sim,
        'real_time_factor': sim / wall if wall > 0 else None,
        'trials_per_minute': len(trials) / (wall / 60) if wall > 0 else None,
        'provenance': provenance(),
    }
    write_outputs(run_dir, trials, run_info)
    print(f'results: {run_dir}')
    node.destroy_node()
    rclpy.try_shutdown()
    return EXIT_OK if complete else EXIT_UNRESPONSIVE


def main(argv=None) -> int:
    """Parse arguments and run a subcommand."""
    parser = argparse.ArgumentParser(prog='benchmark', description=__doc__.splitlines()[1])
    sub = parser.add_subparsers(dest='command', required=True)
    run_parser = sub.add_parser('run', help='run trials against the live simulation')
    run_parser.add_argument('--task', choices=('all', 'reach', 'pick_place'), default='all')
    run_parser.add_argument('--trials', type=int, default=100)
    run_parser.add_argument('--seed', type=int, default=42)
    run_parser.add_argument('--out', default='results')
    compare_parser = sub.add_parser('compare', help='compare two run directories')
    compare_parser.add_argument('a', type=Path)
    compare_parser.add_argument('b', type=Path)
    # Launch appends ROS arguments; ignore them.
    args, _ = parser.parse_known_args(argv)
    if args.command == 'compare':
        print(compare_dirs(args.a, args.b), end='')
        return EXIT_OK
    return run(args)
