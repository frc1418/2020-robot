import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition, TRAJECTORIES
from common.rev import CANSparkMax
from common.limelight import Limelight
from components import Align, Drive, Odometry, Follower, Intake, Launcher
from . import follower_state


class Charge(AutonomousStateMachine):
    MODE_NAME = 'Charge'

    drive: Drive
    follower: Follower
    odometry: Odometry
    intake: Intake
    limelight: Limelight
    launcher: Launcher

    def on_enable(self):
        self.odometry.reset(TRAJECTORIES['trench'].start)
        self.logger.info(f'{self.odometry.get_pose_string()}')
        self.shot_count = 0
        self.completed_trench = False
        super().on_enable()

    @timed_state(first=True, duration=10, next_state='align')
    def rotate_test(self, state_tm):
        pass

    @state
    def align(self):
        self.drive.set_target(0, relative=False)

        if self.drive.angle_setpoint is not None:
            self.drive.align()
        if self.drive.target_locked:
            self.next_state('trench_move')

    @state
    def trench_move(self, tm, state_tm, initial_call):
        self.follower.follow_trajectory('trench', state_tm)
        self.shot_count = 0
        self.intake.spin(-1)
        # if self.follower.is_finished('trench'):
            # self.done()

