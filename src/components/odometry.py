from wpilib.kinematics import DifferentialDriveKinematics, DifferentialDriveOdometry
import navx


class Odometry:
    kinematics: DifferentialDriveKinematics
    navx: navx.AHRS

    def getAngle(self):
        """Return the navx angle with counter-clockwise as positive"""
        return -self.navx.getAngle()

    def setup(self):
        self.odometry = DifferentialDriveOdometry(self.getAngle())
        pass

    def execute(self):
        # TODO: Use encoders ot measure left and right distance meters
        self.odometry.update(self.getAngle(), 0, 0)
        pass
