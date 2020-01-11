import magicbot
import wpilib
import wpilib.drive
import math
from magicbot import tunable

from rev import CANSparkMax, MotorType
from ColorSensorV3 import ColorSensorV3
from ColorMatch import ColorMatch


class TestRobot(magicbot.MagicRobot):

    
    colorString = tunable("", writeDefault=False)
    colorMatcher = ColorMatch()

    BlueTarget = ColorSensorV3.RawColor(0, 255, 255, 0)
    GreenTarget = ColorSensorV3.RawColor(0, 255, 0, 0)
    RedTarget = ColorSensorV3.RawColor(255, 0, 0, 0)
    YellowTarget = ColorSensorV3.RawColor(0, 255, 255, 0)

    def createObjects(self):
        # self.left_joystick = wpilib.Joystick(0)
        # self.right_joystick = wpilib.Joystick(1)
        # self.alt_joystick = wpilib.Joystick(2)

        # self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(10, MotorType.kBrushed), CANSparkMax(20, MotorType.kBrushed))
        # self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(30, MotorType.kBrushed), CANSparkMax(40, MotorType.kBrushed))

        # self.drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)

        self.colorMatcher.addColorMatch(self.BlueTarget)
        self.colorMatcher.addColorMatch(self.GreenTarget)
        self.colorMatcher.addColorMatch(self.RedTarget)
        self.colorMatcher.addColorMatch(self.YellowTarget)
    def teleopPeriodic(self):
        detectedColor = self.colorSensor.getColor()

        match = self.colorMatcher.matchClosestColor(detectedColor)
        if match.color == self.BlueTarget:
            self.colorString = "Blue"
        elif match.color == self.RedTarget:
            self.colorString = "Red"
        elif match.color == self.GreenTarget:
            self.colorString = "Green"
        elif match.color == self.YellowTarget:
            self.colorString = "Yellow"
        else:
            self.colorString = "Unknown"

if __name__ == '__main__':
    wpilib.run(TestRobot)
