from functools import partial, wraps
from logging import Logger
from typing import (Any, Callable, ClassVar, Optional, Protocol, Type, TypeVar,
                    runtime_checkable)

from magicbot import AutonomousStateMachine, state

from components import Follower


class FollowerComponent(Protocol):
    follower: Follower

T = TypeVar('T', bound=FollowerComponent)
StateFunction = Callable[[T, float, float, bool], None]

def follower_state(
    *, 
    trajectory_name: str, 
    next_state: Optional[str] = None, 
    first: bool = False, 
    must_finish: bool = False
) -> Callable[[StateFunction], StateFunction]:
    def decorator(f: StateFunction) -> StateFunction:
        @wraps(f)
        def wrapper(self: FollowerComponent, tm: float, state_tm: float, initial_call: bool) -> None:
            if initial_call:
                self.logger.info(f'Starting Trajectory {trajectory_name}')  # type: ignore

            self.follower.follow_trajectory(trajectory_name, state_tm)
            f(self, tm, state_tm, initial_call)
            if self.follower.is_finished(trajectory_name):
                self.logger.info(f'Finished trajectory: {trajectory_name}')  # type: ignore
                if next_state is not None:
                    self.next_state(next_state)  # type: ignore
                else:
                    self.done()  # type: ignore
        return state(wrapper, first=first, must_finish=must_finish)
    return decorator
