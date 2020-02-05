import wpilib
from common.rev import CANSparkMax
from magicbot import will_reset_to
from magicbot import tunable


class Climber:
    climber_motor: CANSparkMax
    climber_solenoid: wpilib.DoubleSolenoid
    climber_on = will_reset_to(False)
    climber_piston = will_reset_to(True)
    climber_traveled = will_reset_to(0)
    # ENCODER_TICKS_PER_REVOLUTION = 55000

    def raise_scissor(self):
        self.climber_piston = False

    def lower_scissor(self):
        self.climber_piston = True

    def winch_motor_on(self):
        self.climber_piston = True

    def execute(self):
        if self.climber_piston:
            self.climber_solenoid = wpilib.DoubleSolenoid.Value.kForward
        else:
            self.climber_solenoid = wpilib.DoubleSolenoid.Value.kReverse

        if self.climber_on:
            self.climber_motor.set(1)
        else:
            self.climber_motor.set(0)
