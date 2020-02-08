from wpilib.kinematics import DifferentialDriveKinematics, DifferentialDriveOdometry
from wpilib.geometry import Rotation2d
from common.rev import CANEncoder
from networktables.util import ntproperty
import math
import navx


class Odometry:
    kinematics: DifferentialDriveKinematics
    navx: navx.AHRS
    left_encoder: CANEncoder
    right_encoder: CANEncoder
    left_distance = ntproperty('/left distance/', 0)
    right_distance = ntproperty('/right distance/', 0)

    def getAngle(self):
        """Return the navx angle with counter-clockwise as positive"""
        return -self.navx.getAngle()

    def setup(self):
        self.odometry = DifferentialDriveOdometry(Rotation2d(math.radians(self.getAngle())))

    def get_pose(self):
        self.odometry.getPose()

    def get_distance(self, left=True):
        if left:
            return self.left_distance
        else:
            return self.right_distance

    def execute(self):
        self.odometry.update(Rotation2d(math.radians(self.getAngle())), self.left_encoder.getPosition(), self.right_encoder.getPosition())
        self.left_distance = self.left_encoder.getPosition()
        self.right_distance = self.right_encoder.getPosition()
