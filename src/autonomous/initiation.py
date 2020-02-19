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

    @follower_state(trajectory_name='trench', next_state='trench-return')
    def trench_move(self, tm, state_tm, initial_call):
        self.logger.info('Following trench')
        self.launcher.setVelocity(1000)
        self.shot_count = 0
        self.intake.spin(-1)

    @follower_state(trajectory_name='trench-return', next_state='align')
    def trench_return(self, tm, state_tm, initial_call):
        self.launcher.setVelocity(1000)
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
        self.logger.info('Spinning up')
        if self.shot_count == 4:
            if self.completed_trench:
                self.done()
            else:
                self.next_state('trench_move')

        # Wait until shooter motor is ready
        self.launcher.setVelocity(2100 if self.completed_trench else 1950)
        if self.launcher.at_setpoint():
            self.next_state('shoot')

    @timed_state(duration=0.5, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        self.logger.info('Shooting')
        self.shot_count += 1

        self.launcher.setVelocity(2100 if self.completed_trench else 1950)

        if state_tm < 0.25:
            self.launcher.fire()
