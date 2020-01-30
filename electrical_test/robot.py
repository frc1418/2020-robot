import magicbot
import wpilib
import wpilib.drive
import math
import time
import random
from networktables.util import ntproperty
from magicbot import tunable
from wpilib import Spark
# from electrical_test.blinkinLED import BlinkinLED
from robotpy_ext.misc.periodic_filter import PeriodicFilter, logging
# from limelight.py import Limelight
from networktables.util import ntproperty

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

#     red = tunable(0, writeDefault=False)
#     green = tunable(0, writeDefault=False)
#     blue = tunable(0, writeDefault=False)
#     currentColorString = tunable("", writeDefault=False)
#     turnToColorString = tunable("", writeDefault=False)
#     fmsColorString = tunable("", writeDefault=False)
    
    def createObjects(self):
        # self.LED = wpilib.Spark(8)
        # self.RAINBOW = -0.89
        # self.HEARTBEAT_RED = -0.35
        # self.GREEN = 0.71
        self.limelight = Limelight()
        self.i = 0

        # self.leds = BlinkinLED(3)

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
    

    def teleopPeriodic(self):
        # self.leds.set(0.71)
        # print(self.limelight.findPlaneDistance())
       
        # 0: LED mode in pipeline
        # 1: force off
        # 2: force blink
        # 3: force on
        if (self.i <= 30):
            self.limelight.setLightMode(3)
        else:
            self.limelight.setLightMode(1)
        self.i += 1




class BlinkinLED(Spark):
    RAINBOW = -0.89
    HEARTBEAT_RED = -0.35
    GREEN = 0.71

    def __init__(self, channel: int) -> None:
        super().__init__(channel)
  
        # full of balls = heartbeat red (-.35)
        # able to pick up balls = lawn green (.71)
        # climbing = rainbow with glitter (-.89)
        # color scanning = OFF


class Limelight():
    yaw_angle = ntproperty('/limelight/tx',0)
    pitch_angle = ntproperty('/limelight/ty',0)
    light_mode = ntproperty('/limelight/ledMode', 0)
    valid_target = ntproperty('limelight/tv', 0)
    camera_mode = ntproperty('limelight/camMode', 0)

    # change with new robot; UNIT = inches
    CAMERA_HEIGHT = 12 
    TARGET_HEIGHT = 98.25
    # UNIT = degrees
    CAMERA_ANGLE = 20

    def findPlaneDistance(self):
        if self.valid_target == 0:
            return 0
        else:
            plane_distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / math.tan(math.radians(self.CAMERA_ANGLE + self.pitch_angle))
            return plane_distance

    def findDirectDistance(self):
        if valid_target == 0:
            return 0
        else:
            direct_distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT)/math.sin(math.radians(self.CAMERA_ANGLE + self.pitch_angle))
            return direct_distance

    def getYaw(self):
        return self.yaw_angle

    def setLightMode(self, mode: int):
        # 0: LED mode in pipeline
        # 1: force off
        # 2: force blink
        # 3: force on
        self.light_mode = mode

    def setCamMode(self, mode: bool):
        # true: Driver Camera
        # false: Vision Processor
        self.camera_mode = int(mode)

    

if __name__ == '__main__':
    wpilib.run(TestRobot)
