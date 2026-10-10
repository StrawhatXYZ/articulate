"""
One instruction from words to motion: hear, plan, check, execute, log.

Shared by the ROS action server and the end-to-end benchmark, so what is
measured is exactly what people talk to. Never raises: unexpected errors
become a failed outcome.
"""

import json
from pathlib import Path
import threading
import time
import traceback
from typing import Callable, List, NamedTuple, Optional

from lab_simulation.plan_check import check, describe_step
from lab_simulation.planner import describe_scene
from lab_simulation.tasks import SimulationUnresponsive

OUTCOMES = ('done', 'answered', 'refused', 'not_understood', 'failed',
            'planner_unavailable', 'cancelled')


class InstructResult(NamedTuple):
    """The outcome of one instruction."""

    outcome: str
    message: str
    steps: List[str]
    record: dict


class Instructor:
    """Runs instructions one at a time."""

    def __init__(self, planner, executor, log_path: Optional[Path] = None):
        self._planner, self._executor, self._log = planner, executor, log_path
        self._lock = threading.Lock()
        self._busy = False

    def try_begin(self) -> bool:
        """Claim the robot; False if an instruction is already running."""
        with self._lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def end(self) -> None:
        """Release the robot."""
        with self._lock:
            self._busy = False

    def run(self, text: str, feedback: Callable[[str, str], None],
            cancelled: Callable[[], bool]) -> InstructResult:
        """Carry out text; return the outcome."""
        record = {'text': text, 'time': time.time()}
        try:
            result = self._run(text, feedback, cancelled, record)
        except SimulationUnresponsive:
            raise                       # the simulation stopped: the caller decides
        except Exception as error:      # noqa: BLE001  the action must always answer
            record['traceback'] = traceback.format_exc()
            result = InstructResult('failed', f'Failed: internal error ({error}).', [], record)
        record.update(outcome=result.outcome, message=result.message, steps=result.steps)
        if self._log is not None:
            self._log.parent.mkdir(parents=True, exist_ok=True)
            with open(self._log, 'a') as out:
                out.write(json.dumps(record, default=str) + '\n')
        return result

    def _run(self, text, feedback, cancelled, record) -> InstructResult:
        feedback('heard', text)
        states, _ = self._executor.observe()
        record['scene'] = describe_scene(states)
        feedback('planning', '')
        reply = self._planner.plan(text, states)
        record.update(planner_seconds=reply.seconds, raw=reply.raw,
                      calls=[[c.name, c.args] for c in reply.calls])
        if reply.error == 'unavailable':
            return InstructResult('planner_unavailable',
                                  'Planner offline: start Ollama and run '
                                  '"ollama pull gemma4:e2b".', [], record)
        if reply.error:
            return InstructResult('not_understood', "Sorry, I didn't understand that.", [],
                                  record)
        verdict = check(reply.calls, states)
        record.update(verdict=verdict.kind, verdict_source=verdict.source)
        if verdict.kind == 'answer':
            return InstructResult('answered', verdict.message, [], record)
        if verdict.kind == 'refuse':
            return InstructResult('refused', f'Refused: {verdict.message}', [], record)
        steps = [describe_step(s) for s in verdict.steps]
        feedback('plan', f'{len(steps)} steps: ' + '; '.join(steps))
        if not verdict.steps:
            why = '; '.join(verdict.skipped) or 'everything is already where that would put it'
            return InstructResult('done', f'Nothing to do: {why}.', [], record)
        outcome, message = self._executor.run(verdict.steps, feedback, cancelled)
        return InstructResult(outcome, message, steps, record)
