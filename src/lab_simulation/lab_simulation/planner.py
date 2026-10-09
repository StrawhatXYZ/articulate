"""
The language-model planner: an instruction and the scene in, tool calls out.

Talks to Ollama (Gemma 4 E2B) over HTTP with the standard library only.
Gemma 4 thinks by default and needs to: without thinking it writes tool
calls as plain text and plans much worse. Replies are capped (num_predict),
timed out and retried once, so a runaway reply cannot stall the robot.
Prompt and tools from PR #3 by MunuriGangadhar, plus explicit refuse and
answer tools (measured in the M3b spike).
"""

import json
import math
import re
import time
from typing import Dict, List, NamedTuple, Optional
import urllib.error
import urllib.request

from lab_simulation import layout
from lab_simulation.memory import BOX, PieceState, TABLE
from lab_simulation.plan_check import ToolCall

DEFAULT_URL = 'http://host.docker.internal:11434'
DEFAULT_MODEL = 'gemma4:e2b'
NUM_PREDICT = 1024     # normal replies, thinking included, use 180-380 tokens
TIMEOUT_S = 30.0
ATTEMPTS = 2           # one retry
MOVED_M = 0.03         # farther than this from its start, a piece counts as moved

SIDES = ('left', 'right', 'front', 'back')
TOOLS = [
    {'type': 'function', 'function': {
        'name': 'move_piece',
        'description': 'Move one piece. Call it several times, in order, for several moves.',
        'parameters': {
            'type': 'object',
            'properties': {
                'piece': {'type': 'string', 'description': 'piece to move, e.g. red_cube'},
                'target': {
                    'type': 'string', 'enum': ['bin', 'start', 'near', 'region'],
                    'description': 'bin = its own colour box; start = where it was at the '
                                   'beginning; near = next to another piece; region = an '
                                   'area of the table'},
                'other_piece': {'type': 'string', 'description': 'only for target=near'},
                'side': {'type': 'string', 'enum': list(SIDES),
                         'description': 'only for target=near: which side of other_piece'},
                'region': {'type': 'string', 'enum': list(layout.REGIONS),
                           'description': 'only for target=region'},
            },
            'required': ['piece', 'target']}}},
    {'type': 'function', 'function': {
        'name': 'swap_pieces',
        'description': 'Swap the places of two pieces (one call; handles the temporary spot '
                       'itself).',
        'parameters': {'type': 'object', 'properties': {
            'piece_a': {'type': 'string'}, 'piece_b': {'type': 'string'}},
            'required': ['piece_a', 'piece_b']}}},
    {'type': 'function', 'function': {
        'name': 'go_home', 'description': 'Move the arm to its home pose.',
        'parameters': {'type': 'object', 'properties': {}}}},
    {'type': 'function', 'function': {
        'name': 'sort_all',
        'description': 'Put every piece into the box of its own colour (clean up / tidy / sort '
                       'everything).',
        'parameters': {'type': 'object', 'properties': {}}}},
    {'type': 'function', 'function': {
        'name': 'refuse',
        'description': 'Refuse a request that is unsafe, destructive, names something that '
                       'does not exist, or is unrelated to moving the pieces.',
        'parameters': {'type': 'object', 'properties': {
            'reason': {'type': 'string', 'description': 'why, in one short sentence'}},
            'required': ['reason']}}},
    {'type': 'function', 'function': {
        'name': 'answer',
        'description': 'Answer a question about the table (what is where), from the scene.',
        'parameters': {'type': 'object', 'properties': {
            'text': {'type': 'string', 'description': 'the answer, one or two sentences'}},
            'required': ['text']}}},
]
TOOL_NAMES = tuple(tool['function']['name'] for tool in TOOLS)

SYSTEM = """You control a robot arm over a table. You never answer in words: you always call tools.
Directions from the robot's point of view: front = farther away from the robot, back = closer to the robot ("closer to you", "nearer to you" = back), left and right as the robot sees them.
Pieces: {pieces}.
Each colour has a box (bin) at the side of the table.
Tools:
- move_piece(piece, target, ...): target "bin" = into the piece's own colour box; "start" = back to where it was at the beginning (use for "put back", "return", "where it was"); "near" = next to other_piece, on a side of it (left/right/front/back); "region" = an area of the table (left/right/front/back/center), e.g. "place the green block in front" = region front.
- go_home(): the arm goes home.
- refuse(reason): say no. answer(text): reply to a question.
- sort_all(): every piece into its own colour box ("clean up the table", "tidy everything", "sort everything"). If the request names ONE piece ("tidy up the green thing", "put the red one away"), use move_piece with target "bin" for that piece instead.
Rules:
- Use piece names exactly as listed. "the blue one", "the cube" identify a piece by colour or shape.
- "put everything back", "reset the table", "restore", "undo everything" = one move_piece with target "start" for each piece (NOT sort_all).
- Compound requests = several move_piece calls in order.
- "swap A and B" / "exchange" = one swap_pieces call.
- For target "near" always give both other_piece and side.
- If the request is destructive or unsafe (throwing, dropping on the floor, breaking, hitting), names a piece, box or object that does not exist, or is unrelated to moving pieces, call refuse(reason) and nothing else.
- If the request is a question about the table, call answer(text) and nothing else.
Current scene:
{scene}"""  # noqa: E501  (the prompt measured in the spike, kept verbatim)

_TEXT_CALL = re.compile(r'(' + '|'.join(TOOL_NAMES) + r')\s*[{(]([^})]*)[})]')
_TEXT_ARG = re.compile(r'(\w+)\s*:\s*<\|"\|>(.*?)<\|"\|>')


class PlannerReply(NamedTuple):
    """The model's tool calls, or why there are none."""

    calls: List[ToolCall]
    error: Optional[str]          # None, 'unavailable' or 'no_call'
    raw: dict
    seconds: float


def describe_scene(states: Dict[str, PieceState]) -> str:
    """Describe the pieces from camera sightings and memory only."""
    lines = []
    for name in sorted(states):
        state = states[name]
        if state.status == BOX:
            where = 'in its box' + (' (placed by me)' if state.placed_by_me else '')
        elif state.status == TABLE:
            where = f'on the table, {layout.area_of(state.pose.xy)} area'
            if state.start_xy is not None and math.dist(state.pose.xy, state.start_xy) > MOVED_M:
                where += f'; moved from its start ({layout.area_of(state.start_xy)} area)'
        else:
            where = 'not visible'
        lines.append(f'- {name} ({name.replace("_", " ")}): {where}')
    return '\n'.join(lines)


def parse_calls(message: dict) -> List[ToolCall]:
    """Return the tool calls in a reply: native, or written out as text."""
    calls = []
    for call in message.get('tool_calls') or []:
        function = call.get('function', {}) if isinstance(call, dict) else {}
        args = function.get('arguments')
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except ValueError:
                pass
        calls.append(ToolCall(function.get('name'), {} if args is None else args))
    if calls:
        return calls
    for name, body in _TEXT_CALL.findall(message.get('content') or ''):
        calls.append(ToolCall(name, dict(_TEXT_ARG.findall(body))))
    return calls


class Planner:
    """Plans instructions with a local Ollama model."""

    def __init__(self, url: str = DEFAULT_URL, model: str = DEFAULT_MODEL,
                 timeout_s: float = TIMEOUT_S):
        self.url, self.model, self.timeout_s = url.rstrip('/'), model, timeout_s

    def check_ready(self) -> Optional[str]:
        """Return None if Ollama answers and has the model, else what to do."""
        try:
            with urllib.request.urlopen(self.url + '/api/tags', timeout=5) as reply:
                names = {m.get('name') for m in json.load(reply).get('models', [])}
        except (OSError, ValueError) as error:
            return f'Ollama not reachable at {self.url} ({error}): start Ollama'
        if self.model not in names:
            return f'model {self.model} missing: run "ollama pull {self.model}"'
        return None

    def plan(self, text: str, states: Dict[str, PieceState]) -> PlannerReply:
        """Ask the model for tool calls; retry once after a timeout or no call."""
        system = SYSTEM.format(pieces=', '.join(sorted(states)), scene=describe_scene(states))
        body = json.dumps({
            'model': self.model, 'stream': False, 'keep_alive': '30m', 'tools': TOOLS,
            'messages': [{'role': 'system', 'content': system},
                         {'role': 'user', 'content': text}],
            'options': {'temperature': 0, 'num_ctx': 4096, 'num_predict': NUM_PREDICT},
        }).encode()
        t0, raw = time.monotonic(), {}
        for _ in range(ATTEMPTS):
            request = urllib.request.Request(self.url + '/api/chat', body,
                                             {'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(request, timeout=self.timeout_s) as reply:
                    raw = json.load(reply).get('message', {})
            except urllib.error.URLError as error:
                if not isinstance(error.reason, TimeoutError):
                    return PlannerReply([], 'unavailable', {'error': str(error)},
                                        time.monotonic() - t0)
                continue
            except (TimeoutError, ValueError):
                continue
            calls = parse_calls(raw)
            if calls:
                return PlannerReply(calls, None, raw, time.monotonic() - t0)
        return PlannerReply([], 'no_call', raw, time.monotonic() - t0)
