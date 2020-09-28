import wpilib
import wpilib.drive
from wpilib.controller import PIDController
from common.navx import navx
from common.limelight import Limelight
from magicbot import will_reset_to
from networktables.util import ntproperty


class Drive:
    """
    Handle robot drivetrain.
    All drive interaction must go through this class.
    """

    limelight: Limelight
    train: wpilib.drive.DifferentialDrive
    navx: navx.AHRS
    left_motors: wpilib.SpeedControllerGroup
    right_motors: wpilib.SpeedControllerGroup

    y = will_reset_to(0.0)
    rot = will_reset_to(0.0)
    aligning = will_reset_to(False)
    deadband = will_reset_to(0.1)

    auto = will_reset_to(False)
    left_voltage = will_reset_to(0)
    right_voltage = will_reset_to(0)

    speed_constant = will_reset_to(1.05)
    rotational_constant = will_reset_to(0.8)
    squared_inputs = False

    angle_p = ntproperty('/align/kp', 0.04)
    angle_i = ntproperty('/align/ki', 0)
    angle_d = ntproperty('/align/kd', 0.0)
    angle_reported = ntproperty('/align/angle', 0)
    angle_to = ntproperty('/align/angle_to', 0)

    def setup(self):
        """
        Run setup code on the injected variables (train)
        """
        self.angle_controller = PIDController(self.angle_p, self.angle_i, self.angle_d)
        self.angle_controller.setTolerance(2, 1)
        self.angle_controller.enableContinuousInput(0, 360)
        self.angle_setpoint = None
        self.calculated_pid = False

    def set_target(self, angle: float, relative=True):
        if relative and angle is not None:
            self.angle_setpoint = (self.angle + angle) % 360
            self.angle_to = self.angle_setpoint
        else:
            self.angle_setpoint = angle

        if angle is not None:
            self.angle_controller.setSetpoint(self.angle_setpoint)
        else:
            self.calculated_pid = False
            self.angle_controller.reset()

    def align(self):
        self.aligning = True
        self.deadband = 0

    def voltageDrive(self, left_voltage, right_voltage):
        self.left_voltage = left_voltage
        self.right_voltage = right_voltage
        self.auto = True
        self.deadband = 0

    def move(self, y: float, rot: float):
        """
        Move robot.
        :param y: Speed of motion in the y direction. [-1..1]
        :param rot: Speed of rotation. [-1..1]
        """
        self.y = y
        self.rot = rot

    @property
    def target_locked(self):
        # self.logger.info(f'SET {self.angle_controller.getSetpoint()} ANG {self.navx.getAngle()}')
        return self.angle_controller.atSetpoint() and self.calculated_pid

    @property
    def angle(self):
        raw = self.navx.getAngle()
        while raw < 0:
            raw += 360
        return raw % 360

    def execute(self):
        """
        Handle driving.
        """
        self.train.setDeadband(self.deadband)
        self.angle_reported = self.angle

        if self.aligning and self.angle_setpoint is not None:
            self.right_motors.setInverted(False)
            if self.angle_controller.atSetpoint() and self.calculated_pid:
                self.train.arcadeDrive(0, 0, squareInputs=False)
                return

            # Use new network tables variables for testing
            self.angle_controller.setP(self.angle_p)
            self.angle_controller.setD(self.angle_d)
            self.angle_controller.setI(self.angle_i)

            output = self.angle_controller.calculate(self.angle)
            self.logger.info(f'Output: {output}, Error: {self.angle_controller.getPositionError()}')
            # Manual I-term zone (15 degrees)
            if abs(self.angle_controller.getPositionError()) <= 3:
                self.angle_controller.setI(self.angle_i)
                # Minumum and Maximum effect of integrator on output
                self.angle_controller.setIntegratorRange(-0.05, 0.05)
            else:
                self.angle_controller.setI(0)
                self.angle_controller.setIntegratorRange(0, 0)

            if output < 0:
                output = max(output, -0.35)
            else:
                output = min(output, 0.35)

            self.train.arcadeDrive(0, output, squareInputs=False)
            self.calculated_pid = True
        elif self.auto:
            self.right_motors.setInverted(True)
            self.right_motors.setVoltage(self.right_voltage)
            self.left_motors.setVoltage(self.left_voltage)
        else:
            self.right_motors.setInverted(False)
            self.train.arcadeDrive(
                self.speed_constant * self.y,
                self.rotational_constant * self.rot,
                squareInputs=self.squared_inputs,
            )

        if not self.limelight.targetExists():
            self.limelight.target_state = 0
        elif self.target_locked:
            self.limelight.target_state = 2
        else:
            self.limelight.target_state = 1
