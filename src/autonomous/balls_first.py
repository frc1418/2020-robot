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
    """
    Back up to pick up two balls, then drive back and shoot all five balls.
    """

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
        start_pos = self.starting_pos
        # if start_pos == 'LIMELIGHT':
        #     self.odometry.reset(self.limelight.getAveragedPose())
        # else:
        self.odometry.reset(TRAJECTORIES['trench-ball-2'].start)

    @follower_state(trajectory_name='trench-ball-2', next_state='trench_return', first=True)
    def move_trench(self, tm, state_tm, initial_call):
        self.intake.spin(-0.53)

    @follower_state(trajectory_name='trench-ball-2-return', next_state='align')
    def trench_return(self, tm, state_tm, initial_call):
        self.launcher.setVelocity(2000)
        self.limelight.TurnLightOn(True)

    @state
    def align(self):
        self.launcher.setVelocity(2000)
        self.limelight.TurnLightOn(True)
        if self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw() + 2, relative=True)

        if self.drive.angle_setpoint is not None:
            self.drive.align()
        if self.drive.target_locked:
            self.next_state('spinup')

    @state
    def spinup(self, state_tm, initial_call):
        self.limelight.TurnLightOn(True)
        if self.shot_count >= 9:
            self.next_state('realign')

        if self.shot_count >= 3:
            self.intake.spin(-0.93)

        # Wait until shooter motor is ready
        self.launcher.setVelocity(4440)
        if self.shot_count >= 3 and state_tm < 0.5:
            return
        if self.launcher.at_setpoint() and (self.launcher.ball_found() or self.shot_count >= 4) and not initial_call:
            self.next_state('shoot')

    @timed_state(duration=0.75, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        self.limelight.TurnLightOn(True)
        if initial_call:
            self.shot_count += 1

        if self.shot_count >= 3:
            self.intake.spin(-0.93)

        self.launcher.setVelocity(4440)

        if state_tm < 0.25:
            self.launcher.fire()

    @state
    def realign(self):
        self.limelight.TurnLightOn(False)
        self.drive.calculated_pid = False
        self.drive.set_target(0, relative=False)

        self.drive.align()
        if self.drive.target_locked:
            self.done()
