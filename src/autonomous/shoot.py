import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition
from common.rev import CANSparkMax
from common.limelight import Limelight
from components import Align, Drive, Odometry, Follower, Intake, Launcher
from . import follower_state


class Shoot(AutonomousStateMachine):
    MODE_NAME = 'Shoot'

    drive: Drive
    follower: Follower
    odometry: Odometry
    intake: Intake
    limelight: Limelight
    right_motors: wpilib.SpeedControllerGroup
    launcher: Launcher

    def on_enable(self):
        self.shot_count = 0
        super().on_enable()

    @state(first=True)
    def align(self):
        if self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw(), relative=True)

        self.drive.align()

        if self.drive.target_locked:
            self.next_state('spinup')

    @state
    def spinup(self, state_tm):
        if self.shot_count == 3:
            self.done()

        # Wait until shooter motor is ready
        self.launcher.setVelocity(-1750)
        if self.launcher.at_setpoint():
            self.next_state('shoot')

    @timed_state(duration=0.5, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        self.logger.info('Shooting')
        if initial_call:
            self.shot_count += 1

        self.launcher.setVelocity(-1750)

        if state_tm < 0.25:
            self.launcher.fire()