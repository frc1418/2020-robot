import math

from magicbot import default_state, state, timed_state
from magicbot.state_machine import StateMachine
from src.components.controlpanel import ControlPanel

class PanelSpinner(StateMachine):

    control_panel: ControlPanel

    def spin_to(self, position=False):
        self.engage('positionControl' if position else 'rotationControl')

    # First here doesn't matter because we use the argument form of engage
    @state(first=True, must_finish=True)
    def rotationControl(self, initial_call):
        if initial_call:
            self.rotations = 0
            self.last_color = self.control_panel.detected_color

        if self.control_panel.detected_color != self.last_color:
            self.rotations += 1
            self.last_color = self.control_panel.detected_color
        self.control_panel.cp_motor.set(0.25)
        if self.rotations >= 18:
            self.done()

    @state(must_finish=True)
    def positionControl(self):
        if self.control_panel.turn_to_color is None or self.control_panel.detected_color is None:
            return

        if self.control_panel.detected_color != self.control_panel.turn_to_color:
            # print(f'Detected: {self.detected_color.red}, {self.detected_color.green}, {self.detected_color.blue}  Towards: {self.turn_to_color.red}, {self.turn_to_color.green}, {self.turn_to_color.blue}')
            direction = math.copysign(1, self.control_panel.colors.index(
                self.control_panel.detected_color) - self.control_panel.colors.index(self.control_panel.turn_to_color))
            self.control_panel.cp_motor.set(direction * 0.17)
        else:
            self.control_panel.cp_motor.set(0)
            self.done()
