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

    @state
    def trench_move(self, tm, state_tm, initial_call):
        if initial_call:
            self.logger.info('Starting trench move')
            self.logger.info(f'Sample time: {state_tm}')
        if state_tm == 0:
            state_tm = 0.01
        self.follower.follow_trajectory('trench-far', state_tm)
        self.launcher.setVelocity(1000)
        self.shot_count = 0
        self.completed_trench = True
        self.intake.spin(-1)
        if self.follower.is_finished('trench-far'):
            self.logger.info('Finished trench move')
            self.next_state('align')

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
            self.drive.set_target(self.limelight.getYaw(), relative=True)

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
    def spinup(self, state_tm):
        if self.shot_count >= 3:
            if self.completed_trench:
                self.done()
                return
            else:
                self.next_state('align_for_trench')
                return

        # Wait until shooter motor is ready
        self.launcher.setVelocity(4281 if self.completed_trench else 3950)
        if self.launcher.at_setpoint(1 if self.completed_trench else 1.5) and self.launcher.ball_found():
            self.next_state('shoot')

    @timed_state(duration=0.75, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        if initial_call:
            self.shot_count += 1

        self.launcher.setVelocity(4281 if self.completed_trench else 3950)

        if state_tm < 0.25:
            self.launcher.fire()
        elif self.shot_count >= 1:
            self.intake.spin(-1)
