from common.rev import CANSparkMax, ControlType
from magicbot import will_reset_to, tunable
import wpilib
from wpilib.controller import SimpleMotorFeedforwardMeters
from wpilib.controller import PIDController
from networktables.util import ntproperty
from common.rev import CANEncoder, CANPIDController

# Angle: 2.85 = 4530 (234 inches)
# Angle: 14.585 = 4530 (138 inches)
# Angle: -2 = 4830

RPM_TABLE = {
    -2: 4830,
    2.85: 4530,
    14.585: 4530
}


class Launcher:
    launcher_motor: CANSparkMax
    launcher_solenoid: wpilib.Solenoid
    launcher_encoder: CANEncoder
    rpm_controller: CANPIDController
    launcher_sensor: wpilib.Ultrasonic

    balls_collected = ntproperty("/components/intake/ballsCollected", 0)
    target_rpm = tunable(0)
    flywheel_rpm = tunable(0)
    filtered_rpm = tunable(0)

    speed = will_reset_to(0)
    decimal = will_reset_to(0)
    control_velocity = will_reset_to(False)
    shoot = will_reset_to(False)

    RPM_KP = tunable(0)
    RPM_KI = tunable(0)
    RPM_KD = tunable(0)
    RPM_IZONE = tunable(0)
    RPM_FF = tunable(0.0002)

    def setup(self):
        self.range_filter = wpilib.MedianFilter(3)
        self.rpm_controller.setP(self.RPM_KP)
        self.rpm_controller.setI(self.RPM_KI)
        self.rpm_controller.setD(self.RPM_KD)
        self.rpm_controller.setIZone(self.RPM_IZONE)
        self.rpm_controller.setFF(self.RPM_FF)
        self.old_kp = self.RPM_KP
        self.old_ff = self.RPM_FF

    def setVelocity(self, speed):
        """
        :param speed: Speed in Rotations per minute
        """
        # TODO: Reset calculated PID to false when another setpoint is set

        self.speed = speed
        self.target_rpm = speed
        self.control_velocity = True
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

    def at_setpoint(self, tolerance=50):
        return self.calculated_pid and abs(self.launcher_encoder.getVelocity()) < tolerance

    def execute(self):
        self.filtered_rpm = self.launcher_encoder.getVelocity()

        if self.control_velocity:
            if self.RPM_FF != self.old_ff or self.RPM_KP != self.old_kp:
                self.rpm_controller.setP(self.RPM_KP)
                self.rpm_controller.setI(self.RPM_KI)
                self.rpm_controller.setD(self.RPM_KD)
                self.rpm_controller.setIZone(self.RPM_IZONE)
                self.rpm_controller.setFF(self.RPM_FF)
                self.old_ff = self.RPM_FF
                self.old_kp = self.RPM_KP
            self.rpm_controller.setReference(self.speed, ControlType.kVelocity)
        else:
            self.launcher_motor.set(self.decimal)
            self.target_rpm = 0

        self.launcher_solenoid.set(self.shoot)
        self.range_filter.calculate(self.launcher_sensor.getRangeInches())
