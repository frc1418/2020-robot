from magicbot import state
from magicbot.state_machine import _create_wrapper
from functools import partial, wraps
from typing import Optional


def follower_state(f=None, *, trajectory_name: str, next_state: Optional[str] = None, first=False, must_finish=False):
    """
    Denotes a state that follows a trajectory and stops when it finishes
    """
    if f is None:
        return partial(follower_state, trajectory_name=trajectory_name,
                       next_state=next_state, first=first, must_finish=must_finish)

    @wraps(f)
    def wrapper(self, tm, state_tm, initial_call):
        if initial_call:
            self.logger.info(f'Starting Trajectory {trajectory_name}')

        self.follower.follow_trajectory(trajectory_name, state_tm)
        f(self, tm, state_tm, initial_call)
        if self.follower.is_finished(trajectory_name):
            self.logger.info(f'Finished trajectory: {trajectory_name}')
            if next_state is not None:
                self.next_state(next_state)
            else:
                self.done()

    return state(wrapper, first=first, must_finish=must_finish)
