import wpilib
import magicbot
from components import Launcher
from magicbot.state_machine import state, timed_state, StateMachine
from typing import Optional


class AutoShoot(StateMachine):
    launcher: Launcher

    def setup(self):
        self.speed = None

    def fire_when_ready(self, speed: Optional[float] = None):
        self.speed = speed
        self.engage()

    @state(first=True)
    def spinup(self, state_tm):
        if self.speed is not None:
            self.launcher.setVelocity(self.speed)

        self.logger.info(f'At: {self.launcher.at_setpoint()} State_tm: {state_tm}')
        if self.launcher.at_setpoint() and state_tm > 0.25:
            self.next_state('shoot')

    @timed_state(duration=0.5, next_state='spinup')
    def shoot(self, state_tm):
        if self.speed is not None:
            self.launcher.setVelocity(self.speed)

        if state_tm < 0.25:
            self.launcher.fire()
