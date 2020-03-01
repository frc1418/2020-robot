from common.rev import CANSparkMax, ControlType
from magicbot import will_reset_to, tunable
import wpilib
from wpilib.controller import SimpleMotorFeedforwardMeters
from wpilib.controller import PIDController
from networktables.util import ntproperty

# Angle: 2.85 = 4530 (234 inches)
# Angle: 14.585 = 4530 (138 inches)
# Angle: -2 = 4830

RPM_TABLE = {
    -2: 4830,
    2.85: 4530,
    14.585: 4530
}


class Launcher:
    launcher_motors: wpilib.SpeedControllerGroup
    launcher_solenoid: wpilib.Solenoid
    launcher_encoder: wpilib.Encoder
    launcher_sensor: wpilib.Ultrasonic

    balls_collected = ntproperty("/components/intake/ballsCollected", 0)
    target_rpm = tunable(0)
    flywheel_rpm = tunable(0)
    filtered_rpm = tunable(0)

    speed = will_reset_to(0)
    decimal = will_reset_to(0)
    control_velocity = will_reset_to(False)
    shoot = will_reset_to(False)

    RPM_KP = tunable(0.3)
    RPM_KI = tunable(0)
    RPM_KD = tunable(0)

    def setup(self):
        self.rpm_controller = PIDController(self.RPM_KP, self.RPM_KI, self.RPM_KD, period=20)
        self.rpm_controller.setTolerance(100, float('inf'))
        self.feedforward = SimpleMotorFeedforwardMeters(1.47, 0.0614, 5.77e-5)
        self.calculated_pid = False
        self.range_filter = wpilib.MedianFilter(3)

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

    def setVelocityFromDistance(self, angle, default):
        # Linear regression: y = -61.8557x + 4706.29
        if angle < 5 and angle > -1:
            a = -61.8557
            b = 4706.29
            self.setVelocity((a * angle) + b)
        else:
            self.setVelocity(default)

    def setPercentOutput(self, decimal):
        self.decimal = decimal

    # def getSpeed(self):
        # return self.encoder.getVelocity()

    def fire(self):
        self.balls_collected = 0
        self.shoot = True

    def ball_found(self):
        return self.range_filter.calculate(self.launcher_sensor.getRangeInches()) <= 3

    def at_setpoint(self, tolerance=1):
        return self.calculated_pid and abs(self.rpm_controller.getPositionError()) < tolerance

    def execute(self):
        self.filtered_rpm = self.launcher_encoder.getRate() * 60

        if self.control_velocity:
            self.rpm_controller.setP(self.RPM_KP)
            self.rpm_controller.setI(self.RPM_KI)
            self.rpm_controller.setD(self.RPM_KD)

            feedforward = self.feedforward.calculate(self.speed, 0)
            output = feedforward + self.rpm_controller.calculate(self.launcher_encoder.getRate())
            self.calculated_pid = True
            self.launcher_motors.setVoltage(min(output, 9))
        else:
            self.launcher_motors.set(self.decimal)
            self.target_rpm = 0

        self.launcher_solenoid.set(self.shoot)
        self.range_filter.calculate(self.launcher_sensor.getRangeInches())
