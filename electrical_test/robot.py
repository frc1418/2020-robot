import magicbot
import wpilib
import wpilib.drive
import math
import time
import random
from magicbot import tunable

from rev import CANSparkMax, MotorType
# from ColorSensorV3 import ColorSensorV3
# from ColorMatch import ColorMatch
# from enum import Enum
# from blinkinLED import BlinkinLED


# class Colors(Enum):
#     Blue = ColorSensorV3.RawColor(
#         0.187, 0.457, 0.376, 0)  # accurate from ~16.2 cm
#     Green = ColorSensorV3.RawColor(
#         0.178, 0.573, 0.252, 0)  # accurate from ~12.7 cm
#     Red = ColorSensorV3.RawColor(
#         0.497, 0.365, 0.143, 0)  # accurate from ~12.2 cm
#     Yellow = ColorSensorV3.RawColor(
#         0.317, 0.557, 0.124, 0)  # default color


class TestRobot(magicbot.MagicRobot):

    red = tunable(0, writeDefault=False)
    green = tunable(0, writeDefault=False)
    blue = tunable(0, writeDefault=False)
    currentColorString = tunable("", writeDefault=False)
    turnToColorString = tunable("", writeDefault=False)
    fmsColorString = tunable("", writeDefault=False)
    
    def createObjects(self):
        # self.LED = wpilib.Spark(8)
        # self.RAINBOW = -0.89
        # self.HEARTBEAT_RED = -0.35
        # self.GREEN = 0.71
        self.launcher_motor = CANSparkMax(7, MotorType.kBrushless)
        self.launcher_motor.restoreFactoryDefaults()
        self.launcher_motor.setOpenLoopRampRate(0)
        self.encoder = self.launcher_motor.getEncoder()
        self.i = 0
        # self.left_joystick = wpilib.Joystick(0)
        # self.right_joystick = wpilib.Joystick(1)
        # self.alt_joystick = wpilib.Joystick(2)

        # self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(10, MotorType.kBrushed), CANSparkMax(20, MotorType.kBrushed))
        # self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(30, MotorType.kBrushed), CANSparkMax(40, MotorType.kBrushed))

        # self.drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)
        # self.last_time = time.time()

        # self.colors = [val for val in Colors.__members__.values()]
        # self.fmsColor = random.choice(self.colors).value

        # self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)

        # self.colorMatcher = ColorMatch()
        
       # self.LED = BlinkinLED(8)
        # self.i = 0

        # for color in self.colors:
        #     self.colorMatcher.addColorMatch(color)
    
    def setup(self):
        # Set period to 3 seconds, set bypass_level to WARN
        self.logger.addFilter(PeriodicFilter(1, bypass_level=logging.WARN))

    def teleopPeriodic(self):
    #     try:
    #         detectedColor = self.colorSensor.getColor()
    #     except IOError:
    #         pass
    #     else:
    #         self.red = detectedColor.red
    #         self.blue = detectedColor.blue
    #         self.green = detectedColor.green

    #         self.currentColorString = self.colorToString(detectedColor)
        self.launcher_motor.set(-1)
        self.logger.info(self.encoder.getVelocity())


        
        
    #     self.fmsColorString = self.colorToString(self.fmsColor)
    #     colorInt = self.colors.index(self.fmsColor)
    #     self.turnToColorString = self.colorToString(
    #         self.colors[(colorInt + 2) % 4])
        # if self.i <= 50:
        #     self.LED.set(self.RAINBOW)
        # elif self.i <= 100:
        #     self.LED.set(self.HEARTBEAT_RED)
        # elif self.i <= 150:
        #     self.LED.set(self.GREEN)
        # else:
        #     self.LED.set(0)
        # self.i += 1


    # def colorToString(self, color: ColorSensorV3.RawColor):
    #     match = self.colorMatcher.matchClosestColor(color)
    #     return Colors(match.color).name
    def disabledInit(self):
        self.launcher_motor.set(0)
if __name__ == '__main__':
    wpilib.run(TestRobot)
