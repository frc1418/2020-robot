import magicbot
import wpilib
import wpilib.drive
import math

from ctre.wpi_talonsrx import WPI_TalonSRX


class TestRobot(magicbot.MagicRobot):
    ENCODER_PULSE_PER_REV = 1024
    WHEEL_DIAMETER = 0.5

    def createObjects(self):
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.lf_motor = WPI_TalonSRX(10)
        self.lr_motor = WPI_TalonSRX(15)
        self.rf_motor = WPI_TalonSRX(20)
        self.rr_motor = WPI_TalonSRX(25)
        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(wpilib.SpeedControllerGroup(self.lf_motor, self.lr_motor), wpilib.SpeedControllerGroup(self.rf_motor, self.rr_motor))

    def teleopPeriodic(self):
        self.train.arcadeDrive(-self.joystick_left.getY(), self.joystick_right.getX())


if __name__ == '__main__':
    wpilib.run(TestRobot)
