import wpilib
from magicbot import AutonomousStateMachine, default_state, timed_state, state
from networktables.util import ntproperty

from entry_points.trajectory_generator import StartingPosition, TRAJECTORIES
from common.rev import CANSparkMax
from common.limelight import Limelight
from components import Align, Drive, Odometry, Follower, Intake, Launcher


class Nothing(AutonomousStateMachine):
    MODE_NAME = 'Nothing'
    """
    Do nothing during auto period.
    """

    drive: Drive

    @timed_state(first=True, duration=0.5)
    def nothing(self):
        pass
