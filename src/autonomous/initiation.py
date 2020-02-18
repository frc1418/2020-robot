import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition
from common.rev import CANSparkMax
from common.limelight import Limelight
from components import Align, Drive, Odometry, Follower, Intake, Launcher
from . import follower_state


class Initiation(AutonomousStateMachine):
    DEFAULT = True
    MODE_NAME = 'Initiation'

    starting_pos = ntproperty('/autonomous/starting_position', StartingPosition.LEFT.name)

    drive: Drive
    follower: Follower
    odometry: Odometry
    intake: Intake
    limelight: Limelight
    launcher: Launcher

    def on_enable(self):
        super().on_enable()
        start_pos = self.starting_pos
        if start_pos == 'LIMELIGHT':
            self.odometry.reset(self.limelight.getAveragedPose())
        else:
            self.odometry.reset(StartingPosition[start_pos].value)
        self.shot_count = 0
        self.completed_trench = False

    @follower_state(trajectory_name='trench-forward', next_state='align')
    def trench_move(self, tm, state_tm, initial_call):
        self.shot_count = 0
        self.intake.spin(-1)
        self.completed_trench = True

    @state(first=True)
    def align(self):
        if self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw(), relative=True)

        if self.drive.angle_setpoint is not None:
            self.drive.align()

        if self.drive.target_locked:
            self.next_state('spinup')

    @state
    def spinup(self, state_tm):
        if self.shot_count == 3:
            if self.completed_trench:
                self.done()
            else:
                self.next_state('trench_move')

        # Wait until shooter motor is ready
        self.launcher.setVelocity(1800)
        if self.launcher.at_setpoint():
            self.next_state('shoot')

    @timed_state(duration=0.5, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        if initial_call:
            self.shot_count += 1

        self.launcher.setVelocity(1800)

        if state_tm < 0.25:
            self.launcher.fire()
