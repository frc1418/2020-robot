import wpilib
from wpilib.interfaces import SpeedController
from magicbot import will_reset_to
from common.ctre import WPI_VictorSPX


class Intake:
    intake_motor: WPI_VictorSPX
    intake_wheels: WPI_VictorSPX

    speed = will_reset_to(0)

    def spin(self, speed: float):
        self.speed = speed

    def execute(self):
        self.intake_motor.set(self.speed)
        self.intake_wheels.set(-self.speed)
