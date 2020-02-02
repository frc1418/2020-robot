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

COLORS = {
    'B': wpilib.Color(0.187, 0.457, 0.376),  # accurate from ~16.2 cm
    'G': wpilib.Color(0.178, 0.573, 0.252),  # accurate from ~12.7 cm
    'R': wpilib.Color(0.497, 0.365, 0.143),  # accurate from ~12.2 cm
    'Y': wpilib.Color(0.317, 0.557, 0.124),  # accurate from ~20 cm
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
        self.colors = list(colors.values())
        self.fms_color = None
        self.last_color = None

        self.colorMatcher = ColorMatch()

        for color in self.colors:
            self.colorMatcher.addColorMatch(color)

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
        except IOError as e:
            pass
        else:
            # Sometimes black is returned but we don't want it
            if result_color != wpilib.Color.kBlack:
                self.detected_color = result_color

        self.cp_motor.set(self.speed)
        self.cp_solenoid.set(self.solenoid_state)

    @state(first=True, must_finish=True)  # First here doesn't matter because we use the argument form of engage
    def rotationControl(self, initial_call):
        if initial_call:
            self.rotations = 0
            self.last_color = self.detected_color

        if self.detected_color != self.last_color:
            self.rotations += 1
            self.last_color = self.detected_color
        self.cp_motor.set(0.5)
        if self.rotations >= 18:
            self.done()

    @state(must_finish=True)
    def positionControl(self):
        if self.turn_to_color is None or self.detected_color is None:
            return

        if self.detected_color != self.turn_to_color:
            direction = math.copysign(1, self.colors.index(self.detected_color) - self.colors.index(self.fms_color))
            self.cp_motor.set(direction * 0.5)
        else:
            self.done()
