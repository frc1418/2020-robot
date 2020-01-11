import magicbot
import wpilib
import wpilib.drive
import math
import time
from magicbot import tunable

from rev import CANSparkMax, MotorType
from ColorSensorV3 import ColorSensorV3
from ColorMatch import ColorMatch


class TestRobot(magicbot.MagicRobot):

    red = tunable(0, writeDefault=False)
    green = tunable(0, writeDefault=False)
    blue = tunable(0, writeDefault=False)
    colorString = tunable("", writeDefault=False)

    BlueTarget = ColorSensorV3.RawColor(0.187, 0.457, 0.376, 0) # accurate from ~16.2 cm
    GreenTarget = ColorSensorV3.RawColor(0.178, 0.573, 0.252, 0) # accurate from ~12.7 cm
    RedTarget = ColorSensorV3.RawColor(0.497, 0.365, 0.143, 0) # accurate from ~12.2 cm
    YellowTarget = ColorSensorV3.RawColor(0.317, 0.557, 0.124, 0) # default color

    def createObjects(self):
        # self.left_joystick = wpilib.Joystick(0)
        # self.right_joystick = wpilib.Joystick(1)
        # self.alt_joystick = wpilib.Joystick(2)

        # self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(10, MotorType.kBrushed), CANSparkMax(20, MotorType.kBrushed))
        # self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(30, MotorType.kBrushed), CANSparkMax(40, MotorType.kBrushed))

        # self.drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)
        self.last_time = time.time()
        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)

        self.colorMatcher = ColorMatch()

        self.colorMatcher.addColorMatch(self.BlueTarget)
        self.colorMatcher.addColorMatch(self.GreenTarget)
        self.colorMatcher.addColorMatch(self.RedTarget)
        self.colorMatcher.addColorMatch(self.YellowTarget)

    def teleopPeriodic(self):
        try:
            detectedColor = self.colorSensor.getColor()
        except IOError:
            pass
        else:
            self.red = detectedColor.red
            self.blue = detectedColor.blue
            self.green = detectedColor.green

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
