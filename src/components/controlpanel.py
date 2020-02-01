import wpilib
from rev import CANSparkMax
from magicbot import will_reset_to
from magicbot import tunable
from ColorSensorV3 import ColorSensorV3
from ColorMatch import ColorMatch
from enum import Enum
from magicbot import state
from magicbot.state_machine import StateMachine


class Colors(Enum):
    Blue = wpilib.Color(0.187, 0.457, 0.376, 0)  # accurate from ~16.2 cm
    Green = wpilib.Color(0.178, 0.573, 0.252, 0)  # accurate from ~12.7 cm
    Red = wpilib.Color(0.497, 0.365, 0.143, 0)  # accurate from ~12.2 cm
    Yellow = wpilib.Color(0.317, 0.557, 0.124, 0)  # accurate from ~20 cm


class ControlPanel(StateMachine):
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
        self.fmsColor = wpilib.Color(0, 0, 0, 0)

        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.colorMatcher = ColorMatch()

        for color in self.colors:
            self.colorMatcher.addColorMatch(color)

    def spin(self, position=False):
        self.engage('positionControl' if position else 'rotationControl')

    def stop(self):
        self.done()

    def getFMSColor(self):
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
        # Get the color two sectors after the one the FMS wants to see
        self.turnToColorString = Colors(self.colors[(colorInt + 2) % 4]).name

    def execute(self):
        super().execute()
        if len(self.ds.getGameSpecificMessage()) > 0 and self.getColor is True:
            self.getFMSColor()

        try:
            self.detectedColor = self.colorSensor.getColor()
        except IOError:
            pass
        else:
            self.red = self.detectedColor.red
            self.blue = self.detectedColor.blue
            self.green = self.detectedColor.green

            self.currentColorString = Colors(self.detectedColor).name

        self.cp_motor.set(self.speed)

    @state
    def rotationControl(self, initial_call):
        if initial_call:
            self.rotations = 0
            self.lastColorString = self.currentColorString

        if self.currentColorString != self.lastColorString:
            self.rotations += 1
            self.lastColorString = self.currentColorString
        self.cp_motor.set(0.5)
        if self.rotations >= 18:
            self.done()

    @state
    def positionControl(self):
        if self.currentColorString != self.turnToColorString:
            if abs(self.colors.index(self.detectedColor) - self.colors.index(self.fmsColor)) <= 2:
                self.cp_motor.set(0.5)
            else:
                self.cp_motor.set(-0.5)
        else:
            self.done()