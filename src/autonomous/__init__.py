from magicbot import state
from magicbot.state_machine import _create_wrapper
from functools import partial
from typing import Optional


def follow_path(f=None, *, trajectory_name: str, next_state: Optional[str] = None):
    """
    Denotes a state that follows a trajectory and stops when it finishes
    """
    if f is None:
        return partial(follow_path, trajectory_name=trajectory_name, next_state=next_state)

    def wrapper(self, tm, state_tm, initial_call):
        if state_tm == 0.0:
            state_tm = 0.01
        self.logger.info(f'Starting Trajectory {trajectory_name}')
        self.follower.follow_trajectory(trajectory_name, state_tm)
        f(self, tm, state_tm, initial_call)
        if self.follower.is_finished(trajectory_name):
            self.logger.info(f'Finished trajectory: {trajectory_name}')
            if next_state is not None:
                self.next_state(next_state)
            else:
                self.done()

    return wrapper
