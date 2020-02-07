import math
from enum import Enum

import wpilib
from magicbot import default_state, state, timed_state, will_reset_to
from magicbot.state_machine import StateMachine
from networktables.util import ntproperty
# from magicbot import tunable
from rev.color import ColorMatch, ColorSensorV3
from wpilib import DoubleSolenoid

from common.rev import CANSparkMax


class Color:
    def __init__(self, red, green, blue):
        self.red = int(red * 1000) / 1000
        self.green = int(green * 1000) / 1000
        self.blue = int(blue * 1000) / 1000

    @classmethod
    def from_wpilib(cls, color: wpilib.Color):
        return cls(color.red, color.green, color.blue)

    def __repr__(self):
        return f'Color({self.red}, {self.green}, {self.blue})'

    def __eq__(self, other):
        if not isinstance(other, Color) and not isinstance(other, wpilib.Color):
            return False
        if isinstance(other, wpilib.Color):
            other = Color(other.red, other.green, other.blue)
        return self.red == other.red and self.green == other.green and self.blue == other.blue


COLORS = {
    'B': Color(0.187, 0.457, 0.376),  # accurate from ~16.2 cm
    'G': Color(0.178, 0.573, 0.252),  # accurate from ~12.7 cm
    'R': Color(0.497, 0.365, 0.143),  # accurate from ~12.2 cm
    'Y': Color(0.317, 0.557, 0.124),  # accurate from ~20 cm
}


class ControlPanel(StateMachine):
    cp_motor: CANSparkMax
    cp_solenoid: wpilib.DoubleSolenoid
    colorSensor: ColorSensorV3

    solenoid_state = will_reset_to(DoubleSolenoid.Value.kReverse)
    speed = will_reset_to(0)

    def setup(self):
        self.ds = wpilib.DriverStation.getInstance()
        self.turn_to_color = None
        self.detected_color = None
        self.colors = list(COLORS.values())
        self.fms_color = None
        self.last_color = None
        self.resting_timestamp = None

        self.colorMatcher = ColorMatch()

        for color in self.colors:
            self.colorMatcher.addColorMatch(wpilib.Color(color.red, color.green, color.blue))

    def spin(self, speed: int):
        self.speed = speed

    def spin_to(self, position=False):
        self.engage('positionControl' if position else 'rotationControl')

    def set_solenoid(self, extend: bool):
        self.solenoid_state = DoubleSolenoid.Value.kForward if extend else DoubleSolenoid.Value.kReverse

    def getFMSColor(self):
        fms_color_code = self.ds.getGameSpecificMessage()
        self.fms_color = COLORS[fms_color_code]

        color_index = self.colors.index(self.fms_color)
        # Get the color two sectors after the one the FMS wants to see
        self.turn_to_color = self.colors[(color_index + 2) % len(self.colors)]

    def execute(self):
        super().execute()
        if len(self.ds.getGameSpecificMessage()) > 0 and self.fms_color is None:
            self.getFMSColor()

        try:
            # The "confidence" argument here is meant to be a double reference, so it
            # would be set to the confidence in C++ code. Unused here.
            result_color = self.colorMatcher.matchClosestColor(self.colorSensor.getColor(), confidence=0)
        except IOError:
            pass
        else:
            # Sometimes black is returned but we don't want it
            if Color.from_wpilib(result_color) != wpilib.Color.kBlack:
                self.detected_color = Color.from_wpilib(result_color)
                self.detected_color = list(sorted(self.colors, key=lambda c: self.calculate_distance(c, self.detected_color)))[0]

        # self.cp_motor.set(self.speed)
        self.cp_solenoid.set(self.solenoid_state)

    @staticmethod
    def calculate_distance(color1, color2):
        redDiff = color1.red - color2.red
        greenDiff = color1.green - color2.green
        blueDiff = color1.blue - color2.blue

        return math.sqrt((redDiff * redDiff + greenDiff * greenDiff + blueDiff * blueDiff) / 2)

    @default_state
    def stop(self):
        self.cp_motor.set(0)

    @state(first=True, must_finish=True)  # First here doesn't matter because we use the argument form of engage
    def rotationControl(self, initial_call):
        if initial_call:
            self.rotations = 0
            self.last_color = self.detected_color

        if self.detected_color != self.last_color:
            self.rotations += 1
            self.last_color = self.detected_color
        self.cp_motor.set(0.25)
        if self.rotations >= 18:
            self.done()

    @state(must_finish=True)
    def positionControl(self, state_tm):
        print(state_tm)

        if self.turn_to_color is None or self.detected_color is None:
            return

        if self.detected_color != self.turn_to_color:
            # print(f'Detected: {self.detected_color.red}, {self.detected_color.green}, {self.detected_color.blue}  Towards: {self.turn_to_color.red}, {self.turn_to_color.green}, {self.turn_to_color.blue}')
            direction = math.copysign(1, self.colors.index(self.detected_color) - self.colors.index(self.turn_to_color))
            self.cp_motor.set(direction * 0.17)
            self.resting_timestamp = None
        else:
            if self.resting_timestamp is None:
                self.resting_timestamp = state_tm

            # Wait on the correct color for one second
            if state_tm - self.resting_timestamp < 1:
                self.cp_motor.set(0)
            else:
                self.done()
