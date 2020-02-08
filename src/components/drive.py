import wpilib
import wpilib.drive
from wpilib.controller import PIDController
from common.navx import navx
from magicbot import will_reset_to
from networktables.util import ntproperty


class Drive:
    """
    Handle robot drivetrain.
    All drive interaction must go through this class.
    """

    train: wpilib.drive.DifferentialDrive
    navx: navx.AHRS

    y = will_reset_to(0)
    rot = will_reset_to(0)
    aligning = will_reset_to(False)
    deadband = will_reset_to(0.1)

    speed_constant = 1.05
    rotational_constant = 0.5
    squared_inputs = False

    angle_p = ntproperty('/align/kp', 0.0099)
    angle_i = ntproperty('/align/ki', 0)
    angle_d = ntproperty('/align/kd', 0)
    angle_reported = ntproperty('/align/angle', 0)
    angle_to = ntproperty('/align/angle_to', 0)

    def setup(self):
        """
        Run setup code on the injected variables (train)
        """
        self.angle_controller = PIDController(self.angle_p, self.angle_i, self.angle_d)
        self.angle_controller.setTolerance(2, float('inf'))
        self.angle_controller.enableContinuousInput(0, 360)

    def align(self, angle: float, relative=False):
        self.aligning = True
        self.deadband = 0

        if relative:
            self.angle_setpoint = (self.angle + angle) % 360
            self.angle_to = self.angle_setpoint

        self.angle_setpoint = angle

    def move(self, y: float, rot: float):
        """
        Move robot.
        :param y: Speed of motion in the y direction. [-1..1]
        :param rot: Speed of rotation. [-1..1]
        """
        self.y = y
        self.rot = rot

    @property
    def angle(self):
        raw = self.navx.getAngle()
        return raw % 360

    def execute(self):
        """
        Handle driving.
        """
        self.train.setDeadband(self.deadband)
        self.angle_reported = self.angle

        if self.aligning:
            # if self.angle_controller.atSetpoint():
            # print('Stopped aligning')
            # self.train.arcadeDrive(0, 0, squareInputs=False)
            # return

            # Use new network tables variables for testing
            self.angle_controller.setP(self.angle_p)
            self.angle_controller.setD(self.angle_d)
            self.angle_controller.setI(self.angle_i)

            # Manual I-term zone (15 degrees)
            if abs(self.angle_controller.getPositionError()) <= 15:
                self.angle_controller.setI(self.angle_i)
                # Minumum and Maximum effect of integrator on output
                self.angle_controller.setIntegratorRange(-0.15, 0.15)
            else:
                self.angle_controller.setI(0)
                self.angle_controller.setIntegratorRange(0, 0)

            output = self.angle_controller.calculate(self.angle, self.angle_setpoint)
            print(f'Angle: {self.angle} Desired: {self.angle_setpoint} Output: {output} Error: {self.angle_controller.getPositionError()}')
            self.train.arcadeDrive(0, output, squareInputs=False)
        else:
            self.train.arcadeDrive(
                self.speed_constant * self.y,
                self.rot,
                squareInputs=self.squared_inputs,
            )
