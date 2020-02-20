import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition, TRAJECTORIES
from common.rev import CANSparkMax
from common.limelight import Limelight
from components import Align, Drive, Odometry, Follower, Intake, Launcher
from . import follower_state


class Initiation(AutonomousStateMachine):
    DEFAULT = True
    MODE_NAME = 'Initiation'

    drive: Drive
    follower: Follower
    odometry: Odometry
    intake: Intake
    limelight: Limelight
    launcher: Launcher

    def on_enable(self):
        self.odometry.reset(TRAJECTORIES['trench'].start)
        self.shot_count = 0
        self.completed_trench = False
        super().on_enable()

    @state
    def trench_move(self, tm, state_tm, initial_call):
        self.follower.follow_trajectory('trench', state_tm)
        self.launcher.setVelocity(1000)
        self.shot_count = 0
        self.intake.spin(-1)
        if self.follower.is_finished('trench'):
            self.next_state('trench_return')

    @state
    def trench_return(self, tm, state_tm, initial_call):
        self.launcher.setVelocity(1000)
        self.completed_trench = True
        self.follower.follow_trajectory('trench-return', state_tm)
        if self.follower.is_finished('trench-return'):
            self.next_state('align')

    @state(first=True)
    def align(self):
        if self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw(), relative=True)

        if self.drive.angle_setpoint is not None:
            self.drive.align()
        if self.drive.target_locked:
            self.next_state('spinup')

    @state
    def align_for_trench(self):
        self.drive.set_target(0, relative=False)

        if self.drive.angle_setpoint is not None:
            self.drive.align()
        if self.drive.target_locked:
            self.next_state('trench_move')

    @state
    def spinup(self, state_tm):
        if self.shot_count >= 4:
            if self.completed_trench:
                self.done()
                return
            else:
                self.next_state('align_for_trench')
                return

        # Wait until shooter motor is ready
        self.launcher.setVelocity(2100 if self.completed_trench else 1950)
        if self.launcher.at_setpoint():
            self.next_state('shoot')

    @timed_state(duration=0.75, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        if initial_call:
            self.shot_count += 1

        self.launcher.setVelocity(2100 if self.completed_trench else 1950)

        if state_tm < 0.25:
            self.launcher.fire()
        elif self.shot_count == 1:
            self.intake.spin(-1)
