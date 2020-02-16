import wpilib
from networktables.util import ntproperty
from wpilib.interfaces import SpeedController
from magicbot import will_reset_to
from common.ctre import WPI_VictorSPX


class Intake:
    intake_motor: WPI_VictorSPX
    intake_wheels: WPI_VictorSPX
    intake_switch: wpilib.DigitalInput
    intake_solenoid: wpilib.DoubleSolenoid

    balls_collected = ntproperty("/components/intake/ballsCollected", 0)
    speed = will_reset_to(0)
    solenoid_state = will_reset_to(wpilib.DoubleSolenoid.Value.kForward)

    ball_count = 0
    previous_limit = False

    def spin(self, speed: float):
        self.speed = speed

    def extend(self):
        self.set_solenoid(True)

    def retract(self):
        self.set_solenoid(False)

    def set_solenoid(self, extended: bool):
        self.solenoid_state = wpilib.DoubleSolenoid.Value.kReverse if extended else wpilib.DoubleSolenoid.Value.kForward

    def execute(self):
        self.intake_motor.set(self.speed)
        self.intake_wheels.set(-self.speed)

        if self.intake_switch.get():
            if self.intake_switch.get() is not self.previous_limit:
                self.ball_count += 1
        self.previous_limit = self.intake_switch.get()
        self.balls_collected = self.ball_count
        self.intake_solenoid.set(self.solenoid_state)
