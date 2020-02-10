import math

from magicbot import default_state, state, timed_state
from magicbot.state_machine import StateMachine
from components.controlpanel import ControlPanel
from networktables.util import ntproperty


class PanelSpinner(StateMachine):

    control_panel: ControlPanel

    isSpinningPosition = ntproperty('/Controllers/panelSpinner/isSpinningPosition', False)
    isSpinningRotation = ntproperty('/Controllers/panelSpinner/isSpinningRotation', False)

    def spin_to(self, position=False):
        if position:
            self.engage('positionControl')
            self.isSpinningPosition = True
        else:
            self.engage('rotationControl')
            self.isSpinningRotation = True

    # First here doesn't matter because we use the argument form of engage
    @state(first=True, must_finish=True)
    def rotationControl(self, initial_call):
        if initial_call:
            self.rotations = 0
            self.last_color = self.control_panel.detected_color

        if self.control_panel.detected_color != self.last_color:
            self.rotations += 1
            self.last_color = self.control_panel.detected_color
        self.control_panel.spin(0.25)
        if self.rotations >= 18:
            self.isSpinningRotation = False
            self.done()

    @state(must_finish=True)
    def positionControl(self, state_tm):
        if self.control_panel.turn_to_color is None or self.control_panel.detected_color is None:
            return

        if self.control_panel.detected_color != self.control_panel.turn_to_color:
            # print(f'Detected: {self.detected_color.red}, {self.detected_color.green}, {self.detected_color.blue}  Towards: {self.turn_to_color.red}, {self.turn_to_color.green}, {self.turn_to_color.blue}')
            direction = math.copysign(1, self.control_panel.colors.index(
                self.control_panel.detected_color) - self.control_panel.colors.index(self.control_panel.turn_to_color))
            self.control_panel.spin(direction * 0.17)
            self.resting_timestamp = None
        else:
            if self.resting_timestamp is None:
                self.resting_timestamp = state_tm

            # Wait on the correct color for one second
            if state_tm - self.resting_timestamp < 1:
                self.control_panel.spin(0)
            else:
                self.done()
                self.isSpinningPosition = True
