from magicbot import state
from functools import partial, wraps


def follower_state(f=None, *, trajectory_name: str, next_state=None, first=False, must_finish=False):
    """
    Denotes a state that follows a trajectory and stops when it finishes
    """
    if f is None:
        return partial(follower_state, trajectory_name=trajectory_name, first=first, must_finish=must_finish)

    @wraps(f)
    def wrapper(self, initial_call, tm, state_tm):
        self.follower.follow_trajectory(trajectory_name, state_tm)
        f(self, initial_call, tm, state_tm)
        if self.follower.is_finished():
            if next_state is not None:
                self.next_state(next_state)

    return state(wrapper, first=first, must_finish=must_finish)
