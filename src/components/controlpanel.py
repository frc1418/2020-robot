import wpilib
from rev import CANSparkMax
from magicbot import will_reset_to
from magicbot import tunable
from ColorSensorV3 import ColorSensorV3
from ColorMatch import ColorMatch
from enum import Enum


class Colors(Enum):
    Blue = ColorSensorV3.RawColor(
        0.187, 0.457, 0.376, 0)  # accurate from ~16.2 cm
    Green = ColorSensorV3.RawColor(
        0.178, 0.573, 0.252, 0)  # accurate from ~12.7 cm
    Red = ColorSensorV3.RawColor(
        0.497, 0.365, 0.143, 0)  # accurate from ~12.2 cm
    Yellow = ColorSensorV3.RawColor(
        0.317, 0.557, 0.124, 0)  # default color


class ControlPanel:

    red = tunable(0, writeDefault=False)
    green = tunable(0, writeDefault=False)
    blue = tunable(0, writeDefault=False)
    currentColorString = tunable("No Color", writeDefault=True)
    turnToColorString = tunable("No message from field", writeDefault=True)
    fmsColorString = tunable("No message from field", writeDefault=True)

    cp_motor: CANSparkMax

    speed = will_reset_to(0)

    def createObjects(self):
        self.getColor = True

        self.ds = wpilib.DriverStation.getInstance()

        self.colors = [val for val in Colors.__members__.values()]
        self.fmsColor = ColorSensorV3.RawColor(0, 0, 0, 0)

        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.colorMatcher = ColorMatch()

        for color in self.colors:
            self.colorMatcher.addColorMatch(color)

    def spin(self, spin):
        if abs(spin) > 1:
            raise Exception('Speed not valid.')
        self.speed = spin

    def execute(self):
        if len(self.ds.getGameSpecificMessage()) > 0 and self.getColor is True:
            self.fmsColorString = self.ds.getGameSpecificMessage()
            if self.fmsColorString == 'R':
                self.fmsColor = Colors.Red
            elif self.fmsColorString == 'G':
                self.fmsColor = Colors.Green
            elif self.fmsColorString == 'B':
                self.fmsColor = Colors.Blue
            elif self.fmsColorString == 'Y':
                self.fmsColor = Colors.Yellow
            self.getColor = False

            self.fmsColorString = Colors(self.fmsColor).name
            colorInt = self.colors.index(self.fmsColor)
            self.turnToColorString = Colors(
                self.colors[(colorInt + 2) % 4]).name

        try:
            detectedColor = self.colorSensor.getColor()
        except IOError:
            pass
        else:
            self.red = detectedColor.red
            self.blue = detectedColor.blue
            self.green = detectedColor.green

            self.currentColorString = Colors(detectedColor).name

        self.cp_motor.set(self.speed)
