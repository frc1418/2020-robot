import magicbot
import wpilib
import wpilib.drive
import math
from magicbot import tunable

from rev import CANSparkMax, MotorType
from ColorSensorV3 import ColorSensorV3


class TestRobot(magicbot.MagicRobot):

    red = tunable(0, writeDefault=False)
    green = tunable(0, writeDefault=False)
    blue = tunable(0, writeDefault=False)

    def createObjects(self):
        # self.left_joystick = wpilib.Joystick(0)
        # self.right_joystick = wpilib.Joystick(1)
        # self.alt_joystick = wpilib.Joystick(2)

        # self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(10, MotorType.kBrushed), CANSparkMax(20, MotorType.kBrushed))
        # self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(30, MotorType.kBrushed), CANSparkMax(40, MotorType.kBrushed))

        # self.drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
    def teleopPeriodic(self):
        # self.drive.arcadeDrive(-self.left_joystick.getY(), self.left_joystick.getX())
        self.red = self.colorSensor.getRed()
        self.green = self.colorSensor.getGreen()
        self.blue = self.colorSensor.getBlue()

if __name__ == '__main__':
    wpilib.run(TestRobot)
