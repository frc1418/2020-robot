from wpilib.kinematics import DifferentialDriveKinematics, DifferentialDriveOdometry
from wpilib.geometry import Rotation2d
from common.rev import CANEncoder
import math
import navx


class Odometry:
    kinematics: DifferentialDriveKinematics
    navx: navx.AHRS
    left_encoder: CANEncoder
    right_encoder: CANEncoder

    def getAngle(self):
        """Return the navx angle with counter-clockwise as positive"""
        return -self.navx.getAngle()

    def setup(self):
        self.odometry = DifferentialDriveOdometry(Rotation2d(math.radians(self.getAngle())))
        self.left_distance = 0
        self.right_distance = 0

    def get_pose(self):
        self.odometry.getPose()

    def get_distance(self, left = True):
        if left:
            return self.left_distance
        else:
            return self.right_distance

    def execute(self):
        self.odometry.update(Rotation2d(math.radians(self.getAngle())), 0, 0)
        self.left_distance = self.left_encoder.getPosition()
        self.right_distance = self.right_encoder.getPosition()
