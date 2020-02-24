import wpilib
import magicbot
from components import Launcher
from magicbot.state_machine import state, timed_state, StateMachine
from typing import Optional


class AutoShoot(StateMachine):
    launcher: Launcher
    launcher_sensor: wpilib.Ultrasonic

    def setup(self):
        self.speed = None
        self.range_filter = wpilib.MedianFilter(3)

    def fire_when_ready(self, speed: Optional[float] = None):
        self.speed = speed
        self.engage()

    @state(first=True)
    def spinup(self, state_tm):
        if self.speed is not None:
            self.launcher.setVelocity(self.speed)

        if self.launcher.at_setpoint(tolerance=1) and state_tm > 0.25 and self.range_filter.calculate(self.launcher_sensor.getRangeInches()) <= 3:
            self.next_state('shoot')

    @timed_state(duration=0.5, next_state='spinup')
    def shoot(self, state_tm):
        if self.speed is not None:
            self.launcher.setVelocity(self.speed)

        if state_tm < 0.25:
            self.launcher.fire()
