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

    RPM_KP = tunable(0.1)
    RPM_KI = tunable(0.002)
    RPM_KD = tunable(0.008)

    def setup(self):
        self.rpm_controller = PIDController(self.RPM_KP, self.RPM_KI, self.RPM_KD, period=20)
        self.rpm_controller.setTolerance(100, float('inf'))
        self.feedforward = SimpleMotorFeedforwardMeters(0.933, 0.197, 0.0169)
        self.rpm_filter: wpilib.LinearFilter = wpilib.LinearFilter.movingAverage(8)
        self.calculated_pid = False

    def setVelocity(self, speed):
        """
        :param speed: Speed in Rotations per minute
        """
        # TODO: Reset calculated PID to false when another setpoint is set

        self.speed = speed / 60
        self.target_rpm = speed
        self.control_velocity = True
        self.rpm_controller.setSetpoint(self.speed)
        # TODO: make dictionary(?) that lets us find speed of motor depending on distance

    def setPercentOutput(self, decimal):
        self.decimal = decimal

    # def getSpeed(self):
        # return self.encoder.getVelocity()

    def fire(self):
        self.shoot = True

    def at_setpoint(self):
        return self.calculated_pid and (self.rpm_controller.getPositionError()) < 100

    def execute(self):
        current_rpm = self.rpm_filter.calculate(self.flywheel_rpm)
        self.flywheel_rpm = current_rpm * 60

        if self.control_velocity:
            self.rpm_controller.setP(self.RPM_KP)
            self.rpm_controller.setI(self.RPM_KI)
            self.rpm_controller.setD(self.RPM_KD)

            feedforward = self.feedforward.calculate(self.speed, 0)
            # output = feedforward + self.rpm_controller.calculate(current_rpm)
            output = feedforward + self.rpm_controller.calculate(self.launcher_encoder.getRate())
            self.calculated_pid = True
            self.launcher_motors.setVoltage(output)
            self.logger.info(output)
        else:
            self.launcher_motors.set(self.decimal)
            self.target_rpm = 0

        self.launcher_solenoid.set(self.shoot)
