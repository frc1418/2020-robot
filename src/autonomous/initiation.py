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

    starting_pos = ntproperty('/autonomous/starting_position', 'LIMELIGHT')

    drive: Drive
    align: Align
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

    @follower_state(first=True, trajectory_name='trench-first-forward', next_state='align')
    def trench_first_forward(self):
        self.intake.spin(-1)

    @follower_state(trajectory_name='trench-second-forward', next_state='align')
    def trench_second_forward(self):
        self.intake.spin(-1)
        self.completed_trench = True

    @state
    def align(self):
        if self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw(), relative=True)

        if self.drive.angle_setpoint is not None:
            self.drive.align()

        if self.drive.target_locked:
            self.next_state('spinup')

    @state
    def spinup(self, state_tm):
        if self.shot_count == 2:
            if self.completed_trench and self.shot_count == 3:
                self.done()
            elif not self.completed_trench:
                self.next_state('trench_second_forward')

        # Wait until shooter motor is ready
        self.launcher.setVelocity(-2000)
        if self.launcher.at_setpoint():
            self.next_state('shoot')

    @timed_state(duration=0.5, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        if initial_call:
            self.shot_count += 1

        self.launcher.setVelocity(-2000)

        if state_tm < 0.25:
            self.launcher.fire()
