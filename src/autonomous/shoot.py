import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition, TRAJECTORIES
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
    launcher: Launcher

    def on_enable(self):
        self.odometry.reset(TRAJECTORIES['trench'].start)
        self.shot_count = 0
        super().on_enable()

    @timed_state(first=True, duration=0.25, next_state='align')
    def wait_for_target(self):
        self.limelight.TurnLightOn(True)

    @state
    def align(self, initial_call):
        if initial_call:
            self.drive.calculated_pid = False
        if self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw() + 3)

        if self.drive.angle_setpoint is not None:
            self.drive.align()
        if self.drive.target_locked:
            self.next_state('spinup')

    @state
    def spinup(self, state_tm):
        if self.shot_count >= 5:
            self.next_state('move_backward')

        # Wait until shooter motor is ready
        self.launcher.setVelocity(1950)
        if self.launcher.at_setpoint():
            self.next_state('shoot')

    @timed_state(duration=0.75, next_state='spinup')
    def shoot(self, state_tm, initial_call):
        if initial_call:
            self.shot_count += 1

        self.launcher.setVelocity(1950)

        if state_tm < 0.25:
            self.launcher.fire()
        elif self.shot_count >= 1:
            self.intake.spin(-1)

    @timed_state(duration=0.3)
    def move_backward(self):
        self.drive.move(-0.3)