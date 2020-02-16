from common.rev import CANSparkMax, ControlType
from magicbot import will_reset_to, tunable
import wpilib
from wpilib.controller import SimpleMotorFeedforwardMeters
from wpilib.controller import PIDController


class Launcher:
    launcher_motors: wpilib.SpeedControllerGroup
    launcher_solenoid: wpilib.Solenoid
    launcher_encoder: wpilib.Encoder

    target_rpm = tunable(0)
    flywheel_rpm = tunable(0)

    speed = will_reset_to(0)
    decimal = will_reset_to(0)
    control_velocity = will_reset_to(False)
    shoot = will_reset_to(False)

    RPM_KP = tunable(0.6)
    RPM_KI = tunable(0.001)
    RPM_KD = tunable(0.1)

    def setup(self):
        self.rpm_controller = PIDController(self.RPM_KP, self.RPM_KI, self.RPM_KD)
        self.feedforward = SimpleMotorFeedforwardMeters(0.953, 0.19, 0.00966)

    def setVelocity(self, speed):
        """
        :param speed: Speed in Rotations per minute
        """
        self.speed = speed / 60
        self.target_rpm = speed
        self.control_velocity = True
        # TODO: make dictionary(?) that lets us find speed of motor depending on distance

    def setPercentOutput(self, decimal):
        self.decimal = decimal

    # def getSpeed(self):
        # return self.encoder.getVelocity()

    def fire(self):
        self.shoot = True

    def at_setpoint(self):
        return abs(self.rpm_controller.getPositionError()) < 100

    def execute(self):
        self.flywheel_rpm = self.launcher_encoder.getRate() * 60

        if self.control_velocity:
            self.rpm_controller.setP(self.RPM_KP)
            self.rpm_controller.setI(self.RPM_KI)
            self.rpm_controller.setD(self.RPM_KD)

            feedforward = self.feedforward.calculate(self.speed, 0)
            output = feedforward + self.rpm_controller.calculate(self.launcher_encoder.getRate(), self.speed)
            self.launcher_motors.setVoltage(min(12, output))
        else:
            self.launcher_motors.set(self.decimal)
            self.target_rpm = 0

        self.launcher_solenoid.set(self.shoot)
