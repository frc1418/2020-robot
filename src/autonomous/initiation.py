import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition
from common.rev import CANSparkMax
from common.limelight import Limelight
from components import Align, Drive, Odometry, Follower
from . import follower_state


class Initiation(AutonomousStateMachine):
    DEFAULT = True
    MODE_NAME = 'Initiation'

    starting_pos = ntproperty('/autonomous/starting_position', 'CENTER', writeDefault=False)

    drive: Drive
    align: Align
    follower: Follower
    odometry: Odometry
    limelight: Limelight
    # TODO: Use launcher component
    launcher_motors: wpilib.SpeedControllerGroup
    launcher_solenoid: wpilib.Solenoid

    def on_enable(self):
        super().on_enable()
        if StartingPosition[self.starting_pos] == 'LIMELIGHT' and self.limelight.targetExists():
            self.odometry.reset(self.limelight.getPose())
        else:
            self.odometry.reset(StartingPosition[self.starting_pos])
        self.shot_count = 0

    @default_state
    def spinup(self, state_tm):
        if self.shot_count >= 3:
            self.done()

        # TODO: Use RPM to check if spun up
        self.launcher_motors.set(0.75)
        if state_tm > 5 or (state_tm > 1 and self.shot_count > 0):
            self.next_state('shoot')

    @timed_state(duration=1, first=False)
    def shoot(self, state_tm):
        self.launcher_motors.set(0.75)  # Continue to set launcher motor speed
        self.launcher_solenoid.set(True)
        if state_tm > 0.5:
            self.launcher_solenoid.set(False)
        self.shot_count += 1

    @follower_state(trajectory_name='trench', first=True)
    def moveForward(self, tm, state_tm, initial_call):
        pass
