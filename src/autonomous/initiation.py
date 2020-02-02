import wpilib
from magicbot import AutonomousStateMachine, default_state, state, timed_state

from common.rev import CANSparkMax
from components import Align, Drive


class Initiation(AutonomousStateMachine):
    drive: Drive
    align: Align
    # TODO: Use launcher component
    launcher_motor: CANSparkMax
    launcher_solenoid: wpilib.Solenoid

    def on_enable(self):
        self.shot_count = 0

    @default_state
    def spinup(state_tm):
        if self.shot_count >= 3:
            self.done()

        # TODO: Use RPM to check if spun up
        self.launcher_motor.set(0.75)
        if state_tm > 5 or (state_tm > 1 and self.shot_count > 0):
            self.next_state('shoot')

    @timed_state(duration=1)
    def shoot(self, state_tm):
        self.launcher_motor.set(0.75)  # Continue to set launcher motor speed
        self.launcher_solenoid.set(True)
        if state_tm > 0.5:
            self.launcher_solenoid.set(False)
        self.shot_count += 1