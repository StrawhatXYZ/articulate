"""
Score instructions by where the pieces end up, and run the planner-only benchmark.

Pure Python. outcome_ok is the one scorer for both benchmarks: the
planner-only run scores the safety check's dry run of the plan; the
end-to-end run scores the simulator's final poses. Locations are 'box'
(inside its own bin), an (x, y), or None (unknown).
"""

import math
import statistics
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from lab_simulation import layout, targets
from lab_simulation.instruction_cases import Case, HOME, NO_MOTION, REFUSE, SCENES
from lab_simulation.memory import BOX, TABLE
from lab_simulation.metrics import percentile, wilson_interval
from lab_simulation.plan_check import check

START_TOLERANCE_M = 0.03
AT_TOLERANCE_M = 0.03
UNMOVED_M = 0.01

Where = Dict[str, object]


def _moved(a, b) -> bool:
    if a is None or b is None or isinstance(a, str) or isinstance(b, str):
        return a != b
    return math.dist(a, b) > UNMOVED_M


def infeasible(expected, outcome: str, refused_by: str, message: str) -> bool:
    """
    Return True if a valid instruction was refused because the table has no room.

    The check refuses a correct plan when the crowded free strip has no spot
    for it (for instance, the other piece's left is taken). That is the
    table's limit, not a planning mistake, so it is reported separately.
    """
    moving = isinstance(expected, dict) or expected == HOME
    return moving and outcome == 'refused' and refused_by == 'check' and 'no free' in message


def _place_ok(want, name: str, before: Where, after: Where, starts) -> Tuple[bool, str]:
    got = after.get(name)
    if want == 'bin':
        return got == 'box', f'{name} not in its box'
    if not isinstance(got, tuple):
        return False, f'{name} is {got}, not on the table'
    if want == 'start':
        return math.dist(got, starts[name]) <= START_TOLERANCE_M, f'{name} not at its start'
    kind = want[0]
    if kind == 'region':
        return layout.in_region(got, want[1]), f'{name} not in the {want[1]} area'
    if kind == 'at':
        spot = before.get(want[1])
        return (isinstance(spot, tuple) and math.dist(got, spot) <= AT_TOLERANCE_M,
                f'{name} not where {want[1]} was')
    if kind == 'near':
        other = after.get(want[1])
        return (isinstance(other, tuple) and targets.beside(got, other, want[2]),
                f'{name} not {want[2] or "next to"} {want[1]}')
    return False, f'unknown expectation {want}'


def outcome_ok(expected, before: Where, after: Where, starts: Dict[str, Tuple[float, float]],
               outcome: str, message: str, went_home: bool) -> Tuple[bool, str]:
    """Return (passed, why not) for one instruction's result."""
    still = not any(_moved(before.get(n), after.get(n)) for n in layout.PIECES)
    if expected == REFUSE:
        return outcome == 'refused' and still, f'outcome {outcome}'
    if expected == NO_MOTION:
        return outcome in ('refused', 'answered') and still, f'outcome {outcome}'
    if isinstance(expected, tuple):
        words = [w.lower() for w in expected[1]]
        found = (all if expected[0] == 'answer' else any)(w in message.lower() for w in words)
        return outcome == 'answered' and found and still, f'outcome {outcome}: {message}'
    if outcome != 'done':
        return False, f'outcome {outcome}: {message}'
    wants = {} if expected == HOME else expected
    if (expected == HOME or wants.get(HOME)) and not went_home:
        return False, 'did not go home'
    for name in layout.PIECES:
        if name in wants:
            ok, why = _place_ok(wants[name], name, before, after, starts)
        else:
            ok, why = not _moved(before.get(name), after.get(name)), f'{name} moved'
        if not ok:
            return False, why
    return True, ''


def _where(states) -> Where:
    return {n: 'box' if s.status == BOX else s.pose.xy if s.status == TABLE else None
            for n, s in states.items()}


def plan_outcome(reply, verdict) -> str:
    """Return the instruction outcome for a planner reply and the check's verdict."""
    if reply.error == 'unavailable':
        return 'planner_unavailable'
    if reply.error:
        return 'not_understood'
    return {'answer': 'answered', 'refuse': 'refused'}.get(verdict.kind, 'done')


def evaluate(planner, cases: Sequence[Case], set_name: str = 'test',
             on_row: Callable[[dict], None] = lambda row: None) -> List[dict]:
    """
    Plan and check every case; score the dry run. Never aborts on a planner error.

    on_row is called with each row as soon as it is scored (progress).
    """
    rows = []
    for case in cases:
        states = SCENES[case.scene]
        reply = planner.plan(case.text, states)
        verdict = check(reply.calls, states) if not reply.error else None
        outcome = plan_outcome(reply, verdict)
        before = _where(states)
        after = dict(before, **verdict.final) if outcome == 'done' else before
        went_home = outcome == 'done' and any(s.action == 'home' for s in verdict.steps)
        message = verdict.message if verdict else ''
        starts = {n: s.start_xy for n, s in states.items()}
        ok, why = outcome_ok(case.expected, before, after, starts, outcome, message, went_home)
        rows.append({'set': set_name, 'category': case.category, 'text': case.text,
                     'scene': case.scene, 'ok': ok, 'why': '' if ok else why,
                     'outcome': outcome, 'message': message, 'seconds': reply.seconds,
                     'calls': [[c.name, c.args] for c in reply.calls],
                     'refused_by': verdict.source if verdict else '',
                     'infeasible': not ok and infeasible(case.expected, outcome,
                                                         verdict.source if verdict else '',
                                                         message)})
        on_row(rows[-1])
    return rows


def _rate(rows: List[dict]) -> dict:
    n, ok = len(rows), sum(r['ok'] for r in rows)
    low, high = wilson_interval(ok, n)
    return {'n': n, 'successes': ok, 'rate': ok / n if n else None, 'ci95': [low, high]}


def summarize(rows: List[dict]) -> dict:
    """Return per-set and per-category success, latency and error counts."""
    summary = {}
    for set_name in sorted({r['set'] for r in rows}):
        part = [r for r in rows if r['set'] == set_name]
        seconds = [r['seconds'] for r in part]
        summary[set_name] = dict(
            _rate(part),
            infeasible=sum(r['infeasible'] for r in part),
            feasible=_rate([r for r in part if not r['infeasible']]),
            by_category={c: _rate([r for r in part if r['category'] == c])
                         for c in sorted({r['category'] for r in part})},
            seconds={'median': statistics.median(seconds), 'p95': percentile(seconds, 95),
                     'max': max(seconds)},
            outcomes={o: sum(r['outcome'] == o for r in part)
                      for o in sorted({r['outcome'] for r in part})})
    return summary


def _pct(rate: Optional[dict]) -> str:
    if not rate or not rate['n']:
        return '0/0'
    return (f"{rate['successes']}/{rate['n']} ({100 * rate['rate']:.0f}%, "
            f"95% CI {100 * rate['ci95'][0]:.0f}-{100 * rate['ci95'][1]:.0f}%)")


def render_report(summary: dict, rows: List[dict], info: dict) -> str:
    """Return a Markdown report of a planner-only run."""
    lines = ['# Planner-only benchmark', '',
             f"Model `{info.get('model')}`, {info.get('date', '')}. "
             "Scored on the safety check's dry run of each plan.", '']
    for set_name, s in summary.items():
        lines += [f'## {set_name}', '', f'Success: {_pct(s)}', '',
                  f"Infeasible (correct plan, no room on the table): {s['infeasible']}; "
                  f"success on the rest: {_pct(s['feasible'])}", '',
                  f"Latency: median {s['seconds']['median']:.1f} s, "
                  f"p95 {s['seconds']['p95']:.1f} s, max {s['seconds']['max']:.1f} s", '',
                  '| Category | Success |', '|---|---|']
        lines += [f'| {c} | {_pct(r)} |' for c, r in s['by_category'].items()]
        lines += ['', 'Outcomes: ' + ', '.join(f'{o} {n}' for o, n in s['outcomes'].items()), '']
        failed = [r for r in rows if r['set'] == set_name and not r['ok']]
        if failed:
            lines += ['Failures:', '']
            lines += [f"- \"{r['text']}\" ({r['scene']}): {r['why']}" for r in failed]
            lines.append('')
    return '\n'.join(lines)
