import wpilib
from magicbot import AutonomousStateMachine, default_state, state, timed_state
from networktables.util import ntproperty

from common.limelight import Limelight
from common.rev import CANSparkMax
from components import Align, Drive, Follower, Intake, Launcher, Odometry
from entry_points.trajectory_generator import TRAJECTORIES, StartingPosition

from . import StateFunction, follower_state


class Initiation(AutonomousStateMachine):
    DEFAULT = True
    MODE_NAME = 'Initiation'
    """
    Align and shoot 3 balls, then drive back to pick up 3 more balls and shoot again.
    """

    drive: Drive
    follower: Follower
    odometry: Odometry
    intake: Intake
    limelight: Limelight
    launcher: Launcher

    def on_enable(self):
        self.odometry.reset(TRAJECTORIES['trench-far'].start)
        self.shot_count = 0
        self.completed_trench = False
        super().on_enable()

    @follower_state(trajectory_name='trench-far', next_state='trench_move_return')
    def trench_move(self, tm: float, state_tm: float, initial_call: bool) -> None:
        self.shot_count = 0
        self.completed_trench = True
        self.intake.spin(-1, 0.5)

    @follower_state(trajectory_name='trench-far-return', next_state='align')
    def trench_move_return(self, tm: float, state_tm: float, initial_call: bool) -> None:
        pass

    # @state
    # def trench_return(self, tm, state_tm, initial_call):
    #     self.launcher.setVelocity(1000)
    #     self.completed_trench = True
    #     self.follower.follow_trajectory('trench-far-return', state_tm)
    #     if self.follower.is_finished('trench-far-return'):
    #         self.next_state('align')

    @timed_state(first=True, duration=0.25, next_state='align')
    def wait_for_target(self):
        self.limelight.TurnLightOn(True)

    @state
    def align(self, initial_call):
        if initial_call:
            self.drive.calculated_pid = False
        if self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw() - 2 if self.completed_trench else self.limelight.getYaw(), relative=True)

        if self.drive.angle_setpoint is not None:
            self.drive.align()
        if self.drive.target_locked:
            self.next_state('spinup')

    @state
    def align_for_trench(self, initial_call):
        if initial_call:
            self.drive.calculated_pid = False
        self.drive.set_target(0, relative=False)

        if self.drive.angle_setpoint is not None:
            self.drive.align()
        if self.drive.target_locked:
            self.next_state('trench_move')

    @state
    def spinup(self, state_tm, initial_call):
        if self.shot_count >= 3 and not self.launcher.ball_found():
            if self.completed_trench:
                self.done()
                return
            else:
                self.next_state('align_for_trench')
                return

        if self.shot_count >= 1:
            self.intake.spin(-1, 0.8)

        # Wait until shooter motor is ready
        self.launcher.setVelocity(4630 if self.completed_trench else 4430)
        if self.launcher.at_setpoint(1 if self.completed_trench else 1.5) and self.launcher.ball_found() and not initial_call:
            self.next_state('shoot')

    @timed_state(duration=0.75, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        if initial_call:
            self.shot_count += 1

        self.launcher.setVelocity(4630 if self.completed_trench else 4430)

        if state_tm < 0.25:
            self.launcher.fire()
        elif self.shot_count >= 1:
            self.intake.spin(-1, 0.8)
