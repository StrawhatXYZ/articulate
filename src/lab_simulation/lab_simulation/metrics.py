"""
Aggregate trial results into summaries, reports and comparisons.

Pure Python (no ROS). A trial is the dict produced by tasks.TrialResult.
"""

import math
import statistics
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

ERROR_LABELS = {'reach': 'Tip error (m)', 'pick_place': 'Placement error (m)'}


def wilson_interval(successes: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    """Return the Wilson score interval for a success rate; (0, 1) when n is 0."""
    if n == 0:
        return (0.0, 1.0)
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    """Return the p-th percentile of values (p from 0 to 100), or None if empty."""
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * p / 100.0
    low = math.floor(rank)
    high = math.ceil(rank)
    return ordered[low] + (ordered[high] - ordered[low]) * (rank - low)


def _stats(values: Sequence[float], keys: Iterable[str]) -> Dict[str, Optional[float]]:
    out: Dict[str, Optional[float]] = {'n': len(values)}
    for key in keys:
        if not values:
            out[key] = None
        elif key == 'mean':
            out[key] = statistics.fmean(values)
        elif key == 'median':
            out[key] = statistics.median(values)
        else:
            out[key] = percentile(values, 95)
    return out


def summarize_task(trials: List[dict]) -> dict:
    """Return the aggregate metrics for the trials of one task."""
    n = len(trials)
    successes = sum(1 for t in trials if t['success'])
    failures: Dict[str, int] = {}
    for t in trials:
        if t['failure']:
            failures[t['failure']] = failures.get(t['failure'], 0) + 1
    cycle = [t['cycle_time_s'] for t in trials if t['success']]
    errors = [t['error_m'] for t in trials if t['error_m'] is not None]
    lo, hi = wilson_interval(successes, n)
    return {
        'trials': n,
        'successes': successes,
        'success_rate': successes / n if n else None,
        'success_ci': [lo, hi],
        'cycle_time_s': _stats(cycle, ('mean', 'median', 'p95')),
        'error_m': _stats(errors, ('mean', 'median', 'p95')),
        'failures': dict(sorted(failures.items())),
        'disturbed': sum(1 for t in trials if t['disturbed']),
    }


def summarize(trials: List[dict], run_info: dict) -> dict:
    """Return the full summary: run information plus per-task aggregates."""
    tasks = sorted({t['task'] for t in trials})
    return {
        'run': run_info,
        'tasks': {task: summarize_task([t for t in trials if t['task'] == task])
                  for task in tasks},
    }


def _row(metric, a, b, higher_is_better, significant=None):
    delta = None if a is None or b is None else b - a
    ratio = None if a in (None, 0) or b is None else b / a
    better = None
    if delta is not None and delta != 0:
        better = (delta > 0) == higher_is_better
    return {'metric': metric, 'a': a, 'b': b, 'delta': delta, 'ratio': ratio,
            'better': better, 'significant': significant}


# Runs made before these keys were recorded (step 1) used the 3-DOF arm with
# suction.
RUN_DEFAULTS = {'robot': 'simple_arm', 'grasp_mode': 'suction'}


def run_setting(summary: dict, key: str):
    """Return a run setting, with the step 1 value for runs that predate it."""
    return summary['run'].get(key, RUN_DEFAULTS.get(key))


def comparability_warnings(a: dict, b: dict) -> List[str]:
    """Return reasons why summaries a and b are not a like-for-like comparison."""
    warnings = []
    for key, label in (('robot', 'robot'), ('grasp_mode', 'grasp mode')):
        if run_setting(a, key) != run_setting(b, key):
            warnings.append(f'different {label}: {run_setting(a, key)} vs {run_setting(b, key)}')
    for key, label in (('seed', 'seed'), ('trials_per_task', 'trials per task')):
        if a['run'].get(key) != b['run'].get(key):
            warnings.append(f"different {label}: {a['run'].get(key)} vs {b['run'].get(key)}")
    if set(a['tasks']) != set(b['tasks']):
        warnings.append(f"different task set: {sorted(a['tasks'])} vs {sorted(b['tasks'])};"
                        ' throughput is not compared')
    for label, summary in (('A', a), ('B', b)):
        if summary['run'].get('complete') is False:
            warnings.append(f'run {label} is incomplete')
    return warnings


def compare(a: dict, b: dict) -> List[dict]:
    """
    Return metric rows comparing summary a (before) with summary b (after).

    Only success rates are tested for significance (Wilson intervals). Other
    rows carry no significance test and are reported as "not tested".
    """
    rows = []
    for task in sorted(set(a['tasks']) & set(b['tasks'])):
        ta, tb = a['tasks'][task], b['tasks'][task]
        overlap = ta['success_ci'][0] <= tb['success_ci'][1] and \
            tb['success_ci'][0] <= ta['success_ci'][1]
        rows.append(_row(f'{task} success rate', ta['success_rate'], tb['success_rate'],
                         True, significant=not overlap))
        rows.append(_row(f'{task} cycle time mean, successes (s)', ta['cycle_time_s']['mean'],
                         tb['cycle_time_s']['mean'], False))
        rows.append(_row(f'{task} error median, measured trials (m)',
                         ta['error_m'].get('median'), tb['error_m'].get('median'), False))
        rows.append(_row(f'{task} error mean, measured trials (m)', ta['error_m']['mean'],
                         tb['error_m']['mean'], False))
    if set(a['tasks']) == set(b['tasks']):
        for key, label in (('real_time_factor', 'real-time factor'),
                           ('trials_per_minute', 'trials per minute')):
            rows.append(_row(label, a['run'].get(key), b['run'].get(key), True))
    return rows


def _fmt(value: Optional[float], digits: int = 3) -> str:
    return 'n/a' if value is None else f'{value:.{digits}f}'


def render_report(summary: dict) -> str:
    """Return a Markdown report for one summary."""
    run = summary['run']
    lines = [
        '# Benchmark report',
        '',
        f"- Date: {run.get('provenance', {}).get('date', 'unknown')}",
        f"- Commit: {run.get('provenance', {}).get('git_commit', 'unknown')}"
        f" (dirty: {run.get('provenance', {}).get('git_dirty', 'unknown')})",
        f"- Architecture: {run.get('provenance', {}).get('arch', 'unknown')}",
        f"- Robot: {run_setting(summary, 'robot')},"
        f" grasp mode: {run_setting(summary, 'grasp_mode')}",
        f"- Seed: {run.get('seed')}, trials per task: {run.get('trials_per_task')}",
        f"- Complete: {run.get('complete')}",
        f"- Real-time factor: {_fmt(run.get('real_time_factor'), 2)},"
        f" trials per minute: {_fmt(run.get('trials_per_minute'), 1)}",
        '',
        'Cycle times cover successful trials only; errors cover trials where the final'
        ' error was measured (n given for each).',
        '',
        '| Task | Success | 95% CI | Cycle mean (s) | Cycle p95 (s) | Cycle n (successes) |'
        ' Error | Error mean | Error median | Error p95 | Error n (measured) | Failures |'
        ' Disturbed |',
        '|---|---|---|---|---|---|---|---|---|---|---|---|---|',
    ]
    for task, s in summary['tasks'].items():
        failures = ', '.join(f'{k}: {v}' for k, v in s['failures'].items()) or 'none'
        rate = 'n/a' if s['success_rate'] is None else f"{100 * s['success_rate']:.0f}%"
        lines.append(
            f"| {task} | {s['successes']}/{s['trials']} ({rate}) |"
            f" {100 * s['success_ci'][0]:.0f}–{100 * s['success_ci'][1]:.0f}% |"
            f" {_fmt(s['cycle_time_s']['mean'], 2)} | {_fmt(s['cycle_time_s']['p95'], 2)} |"
            f" {s['cycle_time_s'].get('n', 'n/a')} |"
            f" {ERROR_LABELS.get(task, 'Error (m)')} | {_fmt(s['error_m']['mean'], 4)} |"
            f" {_fmt(s['error_m'].get('median'), 4)} | {_fmt(s['error_m']['p95'], 4)} |"
            f" {s['error_m'].get('n', 'n/a')} | {failures} | {s['disturbed']} |")
    return '\n'.join(lines) + '\n'


def render_comparison(rows: List[dict], label_a: str, label_b: str,
                      warnings: Sequence[str] = ()) -> str:
    """Return a Markdown table comparing two runs, preceded by any warnings."""
    lines = [f'> Warning: {warning}' for warning in warnings]
    if warnings:
        lines.append('')
    lines += [f'| Metric | {label_a} | {label_b} | Change | Ratio | Verdict |',
              '|---|---|---|---|---|---|']
    for row in rows:
        if row['significant'] is None:
            verdict = 'not tested'
        elif row['significant'] is False:
            verdict = 'not significant'
        elif row['better'] is None:
            verdict = 'no change'
        else:
            verdict = 'better' if row['better'] else 'worse'
        ratio = 'n/a' if row['ratio'] is None else f"{row['ratio']:.2f}×"
        lines.append(f"| {row['metric']} | {_fmt(row['a'])} | {_fmt(row['b'])} |"
                     f" {_fmt(row['delta'])} | {ratio} | {verdict} |")
    return '\n'.join(lines) + '\n'
