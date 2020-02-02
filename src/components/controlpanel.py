from enum import Enum

import wpilib
from wpilib import DoubleSolenoid

from common.rev import CANSparkMax
# from magicbot import tunable
from common.rev_color import ColorMatch, ColorSensorV3
from magicbot import default_state, state, timed_state, will_reset_to
from magicbot.state_machine import StateMachine
from networktables.util import ntproperty


class Colors(Enum):
    Blue = wpilib.Color(0.187, 0.457, 0.376)  # accurate from ~16.2 cm
    Green = wpilib.Color(0.178, 0.573, 0.252)  # accurate from ~12.7 cm
    Red = wpilib.Color(0.497, 0.365, 0.143)  # accurate from ~12.2 cm
    Yellow = wpilib.Color(0.317, 0.557, 0.124)  # accurate from ~20 cm


class ControlPanel(StateMachine):
    red = 0
    green = 0
    blue = 0
    currentColorString = "No Color"
    turnToColorString = "No message from field"
    fmsColorString = "No message from field"
    cp_motor: CANSparkMax
    cp_solenoid: wpilib.DoubleSolenoid

    solenoid_state = will_reset_to(DoubleSolenoid.Value.kReverse)
    speed = will_reset_to(0)

    def setup(self):
        self.getColor = True

        self.ds = wpilib.DriverStation.getInstance()

        self.colors = [val for val in Colors.__members__.values()]
        self.fmsColor = wpilib.Color(0, 0, 0)

        self.colorSensor = ColorSensorV3()
        self.colorMatcher = ColorMatch()

        for color in self.colors:
            self.colorMatcher.addColorMatch(color)

    def spin(self, position=False):
        self.engage('positionControl' if position else 'rotationControl')

    def set_solenoid(self, extend: bool):
        self.solenoid_state = DoubleSolenoid.Value.kForward if extend else DoubleSolenoid.Value.kReverse

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
        if self.cp_solenoid.get() != self.solenoid_state:
            self.cp_solenoid.set(self.solenoid_state)

    @state(first=True, must_finish=True)  # First here doesn't matter because we use the argument form of engage
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

    @state(must_finish=True)
    def positionControl(self):
        if self.currentColorString != self.turnToColorString:
            if abs(self.colors.index(self.detectedColor) - self.colors.index(self.fmsColor)) <= 2:
                self.cp_motor.set(0.5)
            else:
                self.cp_motor.set(-0.5)
        else:
            self.done()
