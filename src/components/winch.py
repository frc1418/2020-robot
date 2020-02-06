import wpilib
from networktables.util import ntproperty
from common.rev import CANSparkMax
from magicbot import will_reset_to
from magicbot import tunable


class Climber:
    climber_motor: CANSparkMax
    climber_solenoid: wpilib.DoubleSolenoid
    climber_on = will_reset_to(False)
    climber_piston = will_reset_to(True)
    height = ntproperty('/winch/height', 0)
    ENCODER_PULSE_PER_REV = 42

    def raise_scissor(self):
        self.climber_piston = True

    def winch_motor_on(self):
        self.climber_on = True

    def execute(self):
        if self.climber_piston:
            self.climber_solenoid = wpilib.DoubleSolenoid.Value.kForward
        else:
            self.climber_solenoid = wpilib.DoubleSolenoid.Value.kReverse

        if self.climber_on:
            # winch motor is negative
            self.climber_motor.set(-0.5)
        else:
            self.climber_motor.set(0)

