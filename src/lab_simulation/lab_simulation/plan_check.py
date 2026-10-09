"""
The safety check between the language model and the arm.

Pure Python. Turns the model's tool calls into validated steps, or refuses
the whole plan with a reason. It never raises on bad input: whatever the
model returns, the answer is a plan, a refusal or an answer. The model never
supplies coordinates; xy targets are created here only for swaps.
"""

from typing import Dict, NamedTuple, Optional, Sequence, Tuple

from lab_simulation import layout, targets
from lab_simulation.memory import BOX, PieceState, TABLE

MAX_STEPS = 8
TOOLS = ('move_piece', 'swap_pieces', 'sort_all', 'go_home', 'refuse', 'answer')
MOVE_TARGETS = ('bin', 'start', 'near', 'region')
_WORDS = {name: set(name.split('_')) for name in layout.PIECES}


class ToolCall(NamedTuple):
    """One tool call from the model, as parsed."""

    name: object
    args: object


class Step(NamedTuple):
    """One validated step: move a piece to a target, or go home."""

    action: str
    piece: Optional[str] = None
    target: Optional[dict] = None


class Verdict(NamedTuple):
    """The check's decision on a reply."""

    kind: str                                   # 'plan', 'refuse' or 'answer'
    steps: Tuple[Step, ...] = ()
    message: str = ''
    source: str = ''                            # who refused: 'model' or 'check'
    final: Dict[str, object] = {}               # dry-run end state: 'box', (x, y) or None


class _Refusal(Exception):
    pass


def _text(args: dict, key: str, required: bool = True) -> Optional[str]:
    value = args.get(key)
    if value is None or (isinstance(value, str) and value.strip().lower() in ('', 'none')):
        if required:
            raise _Refusal(f'missing {key}')
        return None
    if not isinstance(value, str) or len(value) > 200:
        raise _Refusal(f'{key} must be a short text')
    return value.strip()


def _piece(word: str) -> str:
    normal = word.lower().replace(' ', '_')
    if normal in layout.PIECES:
        return normal
    tokens = set(word.lower().replace('_', ' ').split())
    hits = [name for name, words in _WORDS.items() if tokens & words]
    if len(hits) != 1:
        raise _Refusal(f'which piece is "{word}"?')
    return hits[0]


def describe_step(step: Step) -> str:
    """Return a short description of a step."""
    if step.action == 'home':
        return 'go home'
    t = step.target
    where = {'bin': 'its box', 'start': 'where it started'}.get(t['kind'])
    if t['kind'] == 'near':
        where = f"{t['side'] + ' of ' if t.get('side') else 'next to '}{t['piece']}"
    elif t['kind'] == 'region':
        where = f"the {t['region']} of the table"
    elif t['kind'] == 'xy':
        where = 'a free spot' if t.get('temporary') else 'where the other piece was'
    return f'move {step.piece} to {where}'


def _steps_from(call: ToolCall, states: Dict[str, PieceState]):
    if not isinstance(call.args, dict):
        raise _Refusal('arguments must be an object')
    a = call.args
    if call.name == 'go_home':
        return [Step('home')]
    if call.name == 'sort_all':
        return [Step('move', n, {'kind': 'bin'}) for n in sorted(states)
                if states[n].status == TABLE]
    if call.name == 'swap_pieces':
        return [('swap', _piece(_text(a, 'piece_a')), _piece(_text(a, 'piece_b')))]
    piece = _piece(_text(a, 'piece'))
    kind = (_text(a, 'target') or '').lower()
    if kind not in MOVE_TARGETS:
        raise _Refusal(f'unknown target {kind}')
    if kind == 'near':
        side = _text(a, 'side', required=False)
        if side is not None and side.lower() not in targets.SIDES:
            raise _Refusal(f'unknown side {side}')
        return [Step('move', piece, {'kind': 'near', 'piece': _piece(_text(a, 'other_piece')),
                                     'side': side.lower() if side else None})]
    if kind == 'region':
        region = (_text(a, 'region') or '').lower()
        if region not in layout.REGIONS:
            raise _Refusal(f'unknown area {region}')
        return [Step('move', piece, {'kind': 'region', 'region': region})]
    return [Step('move', piece, {'kind': kind})]


def check(calls: Sequence, states: Dict[str, PieceState]) -> Verdict:
    """Return the verdict on the model's tool calls in the given scene."""
    try:
        return _check(list(calls), states)
    except _Refusal as refusal:
        return Verdict('refuse', message=str(refusal), source='check')


def _check(calls, states) -> Verdict:
    if not calls:
        raise _Refusal('no plan')
    if not all(isinstance(c, ToolCall) and isinstance(c.name, str) and c.name in TOOLS
               for c in calls):
        raise _Refusal('unknown tool')
    names = [c.name for c in calls]
    for special in ('refuse', 'answer'):
        if special in names:
            if len(calls) > 1:
                raise _Refusal('the reply contradicts itself')
            args = calls[0].args if isinstance(calls[0].args, dict) else {}
            key = 'reason' if special == 'refuse' else 'text'
            text = args.get(key) if isinstance(args.get(key), str) else ''
            return Verdict(special, message=text[:500], source='model' if special == 'refuse'
                           else '')
    raw = []
    for call in calls:
        raw.extend(_steps_from(call, states))
    return _dry_run(raw, states)


def _dry_run(raw, states) -> Verdict:
    positions = {n: s.pose.xy for n, s in states.items() if s.status == TABLE}
    location: Dict[str, object] = {n: ('box' if s.status == BOX else positions.get(n))
                                   for n, s in states.items()}
    starts = {n: s.start_xy for n, s in states.items() if s.start_xy is not None}
    steps = []

    def move(piece, target, index):
        if location[piece] is None:
            raise _Refusal(f'step {index}: {piece} is not visible')
        if target['kind'] == 'bin' and location[piece] == 'box':
            return
        xy, note = targets.resolve(target, piece, positions, starts)
        if xy is None:
            raise _Refusal(f'step {index}: {note}')
        steps.append(Step('move', piece, target))
        if target['kind'] == 'bin':
            positions.pop(piece, None)
            location[piece] = 'box'
        else:
            positions[piece] = xy
            location[piece] = xy

    for index, item in enumerate(raw, 1):
        if isinstance(item, Step) and item.action == 'home':
            steps.append(item)
        elif isinstance(item, Step):
            move(item.piece, item.target, index)
        else:
            _, a, b = item
            if a == b or location[a] in (None, 'box') or location[b] in (None, 'box'):
                raise _Refusal(f'step {index}: can only swap two pieces on the table')
            xa, xb = location[a], location[b]
            free = None
            for region in ('center', 'front', 'back', 'right', 'left'):
                spot = targets.region_point(region, [p for n, p in positions.items() if n != a]
                                            + [xa, xb])
                if spot is not None:
                    free = spot
                    break
            if free is None:
                raise _Refusal(f'step {index}: no free spot to swap')
            move(a, {'kind': 'xy', 'xy': free, 'temporary': True}, index)
            move(b, {'kind': 'xy', 'xy': xa}, index)
            move(a, {'kind': 'xy', 'xy': xb}, index)
        if len(steps) > MAX_STEPS:
            raise _Refusal(f'more than {MAX_STEPS} steps')
    return Verdict('plan', tuple(steps), final=dict(location))
