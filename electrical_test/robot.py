import magicbot
import wpilib
import wpilib.drive
import math

from rev import CANSparkMax, MotorType

class TestRobot(magicbot.MagicRobot):

    def createObjects(self):

        self.left_joystick = wpilib.Joystick(0)
        self.right_joystick = wpilib.Joystick(1)
        self.alt_joystick = wpilib.Joystick(2)

        self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(10, MotorType.kBrushed), CANSparkMax(20, MotorType.kBrushed))
        self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(30, MotorType.kBrushed), CANSparkMax(40, MotorType.kBrushed))

        self.drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

    def teleopPeriodic(self):
        self.drive.arcadeDrive(-self.left_joystick.getY(), self.left_joystick.getX())

if __name__ == '__main__':
    wpilib.run(TestRobot)
