from wpilib.kinematics import DifferentialDriveKinematics, DifferentialDriveOdometry
from wpilib.geometry import Rotation2d
import math
import navx


class Odometry:
    kinematics: DifferentialDriveKinematics
    navx: navx.AHRS

    def getAngle(self):
        """Return the navx angle with counter-clockwise as positive"""
        return -self.navx.getAngle()

    def setup(self):
        self.odometry = DifferentialDriveOdometry(Rotation2d(math.radians(self.getAngle())))

    def get_pose(self):
        self.odometry.getPose()

    def execute(self):
        # TODO: Use encoders ot measure left and right distance meters
        self.odometry.update(Rotation2d(math.radians(self.getAngle())), 0, 0)
