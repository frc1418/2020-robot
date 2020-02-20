import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition
from common.rev import CANSparkMax
from common.limelight import Limelight
from components import Align, Drive, Odometry, Follower, Intake, Launcher
from entry_points.trajectory_generator import TRAJECTORIES
from . import follower_state


class BallsFirst(AutonomousStateMachine):
    MODE_NAME = 'BallsFirst'

    starting_pos = ntproperty('/autonomous/starting_position', StartingPosition.LEFT.name)

    drive: Drive
    follower: Follower
    odometry: Odometry
    intake: Intake
    limelight: Limelight
    right_motors: wpilib.SpeedControllerGroup
    launcher: Launcher

    def on_enable(self):
        super().on_enable()
        self.shot_count = 0
        self.suck = True
        start_pos = self.starting_pos
        # if start_pos == 'LIMELIGHT':
        #     self.odometry.reset(self.limelight.getAveragedPose())
        # else:
        self.odometry.reset(TRAJECTORIES['trench'].start)

    @state(first=True)
    def move_trench(self, tm, state_tm, initial_call):
        self.follower.follow_trajectory('trench', state_tm)
        if self.follower.is_finished('trench'):
            self.next_state('trench_return')

        if state_tm < 4 and state_tm > 1:
            self.intake.spin(-1)

    @state
    def trench_return(self, state_tm):
        self.limelight.TurnLightOn(True)
        self.follower.follow_trajectory('trench-return', state_tm)
        if self.follower.is_finished('trench-return'):
            self.next_state('align')

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
        if self.shot_count >= 10:
            self.done()

        # Wait until shooter motor is ready
        self.launcher.setVelocity(1950)
        if self.launcher.at_setpoint():
            self.next_state('shoot')

    @timed_state(duration=0.75, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        if initial_call:
            self.shot_count += 1
            self.suck = not self.suck

        if self.shot_count > 2:
            self.intake.spin(-1 if self.suck else 0.5)

        self.launcher.setVelocity(1950)

        if state_tm < 0.25:
            self.launcher.fire()
