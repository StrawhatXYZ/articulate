"""
Run checked plan steps on the robot, one at a time.

Each step looks at the table again first; if the step is no longer valid
the plan stops and the arm goes home. Cancelling takes effect between
steps. The placement memory is updated from what actually happened.
"""

from typing import Callable, Dict, Sequence, Tuple

from lab_simulation import layout, targets
from lab_simulation.memory import BOX, PieceState, PlacementMemory, TABLE
from lab_simulation.plan_check import describe_step, Step
from lab_simulation.tasks import BIN_RELEASE_RAD, Observation, PiecePose

Feedback = Callable[[str, str], None]
# Pick stages at which the piece has been lifted from where it was.
LIFTED_STAGES = ('carry', 'release', 'retreat', 'home')


class Executor:
    """Executes steps with the pick-and-place skill and keeps the memory."""

    def __init__(self, pick, scene, gripper, arm, memory: PlacementMemory):
        self._pick, self._scene, self._gripper, self._arm = pick, scene, gripper, arm
        self.memory = memory

    def observe(self) -> Tuple[Dict[str, PieceState], Observation]:
        """Look at the table; return the updated states and the observation."""
        observation = self._scene.observe()
        self.memory.observe(observation.poses)
        return self.memory.states(), observation

    def reset_arm(self) -> bool:
        """Let go, open partly and go home ignoring pieces and bins."""
        self._scene.unlatch_all()
        self._gripper.release(BIN_RELEASE_RAD)
        self._scene.update_planning_scene(exclude=list(layout.PIECES), bins=False)
        result = self._arm.home()
        self._scene.update_planning_scene()
        return result.planned and result.reached

    def run(self, steps: Sequence[Step], feedback: Feedback,
            cancelled: Callable[[], bool]) -> Tuple[str, str]:
        """Run steps in order; return (outcome, message)."""
        total = len(steps)
        for index, step in enumerate(steps, 1):
            if cancelled():
                self.reset_arm()
                return 'cancelled', f'Cancelled after step {index - 1}/{total}.'
            label = f'step {index}/{total}'
            feedback('step', f'{label}: {describe_step(step)}')
            failure = self._run_step(step)
            if failure:
                self.reset_arm()
                return 'failed', f'Failed at {label}: {failure}.'
        return 'done', 'Done: ' + '; '.join(describe_step(s) for s in steps) + '.'

    def _run_step(self, step: Step) -> str:
        if step.action == 'home':
            return '' if self.reset_arm() else 'could not go home'
        states, observation = self.observe()
        state = states[step.piece]
        if state.status not in (TABLE, BOX):
            return f'{step.piece} is not visible'
        positions = {n: s.pose.xy for n, s in states.items() if s.status == TABLE}
        starts = {n: s.start_xy for n, s in states.items() if s.start_xy is not None}
        xy, note = targets.resolve(step.target, step.piece, positions, starts)
        if xy is None:
            return note
        details: dict = {}
        to_bin = step.target['kind'] == 'bin'
        try:
            failure = self._pick.pick_and_place(
                layout.PIECES[step.piece], state.pose, xy, details, aim_jaws=to_bin,
                nominal_grip=observation.vision, from_bin=state.status == BOX)
        finally:
            self._scene.unlatch(step.piece)
            self._gripper.release(BIN_RELEASE_RAD)
            self._scene.update_planning_scene()
        if to_bin and 'placed' in details:
            # Put in its bin (even if going home afterwards failed).
            self.memory.placed_in_box(step.piece, PiecePose(*details['placed']))
        elif failure is None or details.get('stage') in LIFTED_STAGES:
            self.memory.moved(step.piece)          # picked up: the camera will find it
        # Otherwise the piece never left where it was (a missed grasp, say):
        # memory keeps it, so a piece in its bin is not lost to the masked camera.
        return f'{failure} (stage {details.get("stage")})' if failure else ''
