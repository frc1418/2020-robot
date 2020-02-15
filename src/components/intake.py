import wpilib
from networktables.util import ntproperty
from wpilib.interfaces import SpeedController
from magicbot import will_reset_to
from common.ctre import WPI_VictorSPX


class Intake:
    intake_motor: WPI_VictorSPX
    intake_wheels: WPI_VictorSPX
    intake_switch: wpilib.DigitalInput

    balls_collected = ntproperty("/components/intake/ballsCollected", 0)
    speed = will_reset_to(0)

    ball_count = 0
    previous_limit = False

    def spin(self, speed: float):
        self.speed = speed

    def execute(self):
        self.intake_motor.set(self.speed)
        self.intake_wheels.set(-self.speed)

        if self.intake_switch.get():
            if self.intake_switch.get() is not self.previous_limit:
                self.ball_count+=1
        self.previous_limit = self.intake_switch.get()
        self.balls_collected = self.ball_count

