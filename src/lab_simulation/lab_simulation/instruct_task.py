"""
End-to-end instruction trials: a sentence in, the simulated pieces scored.

Each trial takes the next end-to-end case, resets the placement memory,
looks at the table (which sets each piece's start), runs the case's setup
steps through the executor (for example, putting a piece in its box so it
can be "put back"), then runs the instruction through the same Instructor
that serves /lab/instruct. Ground truth is used only to score where the
pieces ended up.
"""

import random
from typing import Dict

from lab_simulation import layout
from lab_simulation.instruction_cases import E2E, NO_MOTION, REFUSE
from lab_simulation.instruction_eval import outcome_ok
from lab_simulation.tasks import TrialResult


def _never() -> bool:
    return False


def _quiet(stage: str, message: str) -> None:
    pass


class InstructTask:
    """Carry out the end-to-end instruction cases in turn."""

    name = 'instruct'

    def __init__(self, instructor, executor, scene, now=None):
        self._instructor, self._executor, self._scene = instructor, executor, scene
        self._now = now or getattr(scene, 'now', lambda: 0.0)
        self.memory_resets = 0

    def _truth(self) -> Dict[str, object]:
        """Where every piece is, from the simulator: 'box' if inside its own bin."""
        where = {}
        for name in layout.PIECES:
            xy = self._scene.pose(name).xy
            where[name] = 'box' if layout.BINS[name].contains(xy) else xy
        return where

    def run(self, rng: random.Random, index: int) -> TrialResult:
        """Run case index (modulo the number of cases) on the current scene."""
        case = E2E[index % len(E2E)]
        details = {'case': index % len(E2E), 'text': case.text, 'category': case.category}
        self._executor.memory.reset()
        self.memory_resets += 1
        self._executor.observe()
        starts = {n: self._scene.pose(n).xy for n in layout.PIECES}     # scoring only
        t0 = self._now()
        if case.setup:
            outcome, message = self._executor.run(case.setup, _quiet, _never)
            if outcome != 'done':
                details['setup'] = message
                return TrialResult(self.name, index, False, 'setup', 0.0, details=details)
        before = self._truth()
        result = self._instructor.run(case.text, _quiet, _never)
        self._scene.wait_settled()
        after = self._truth()
        went_home = 'go home' in result.steps
        ok, why = outcome_ok(case.expected, before, after, starts, result.outcome,
                             result.message, went_home)
        details.update(outcome=result.outcome, message=result.message, steps=result.steps,
                       why='' if ok else why,
                       planner_seconds=result.record.get('planner_seconds'),
                       calls=result.record.get('calls'), verdict=result.record.get('verdict'),
                       before=dict(before), after=dict(after))
        failure = None if ok else self._blame(case.expected, result)
        return TrialResult(self.name, index, ok, failure, self._now() - t0, details=details)

    @staticmethod
    def _blame(expected, result) -> str:
        """Which part failed: planner, check, execution or outcome."""
        if result.outcome in ('planner_unavailable', 'not_understood'):
            return 'planner'
        wanted_refusal = expected in (REFUSE, NO_MOTION)
        if (result.outcome == 'refused' and not wanted_refusal
                and result.record.get('verdict_source') == 'check'):
            return 'check'
        if result.outcome in ('failed', 'cancelled'):
            return 'execution'
        return 'outcome' if result.outcome == 'done' else 'planner'
