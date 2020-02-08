import math

import magicbot
import navx
import wpilib
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables, NetworkTablesInstance
from networktables.util import ntproperty
from rev import CANSparkMax, MotorType


class Robot(wpilib.IterativeRobot):
    WHEEL_DIAMETER = 0.1524  # Units: Meters
    # Currently unused
    # ENCODER_PULSE_PER_REV = 42 
    GEARING = 7.56  # 7.56:1 gear ratio

    autoSpeedEntry = ntproperty('/robot/autospeed', 0.0)
    telemetry_entry = ntproperty('/robot/telemetry', [0.0], writeDefault=False)
    rotateEntry = ntproperty('/robot/rotate', False)

    l_encoder_pos = ntproperty('/l_encoder_pos', 0)
    l_encoder_rate = ntproperty('/l_encoder_rate', 0)
    r_encoder_pos = ntproperty('/r_encoder_pos', 0)
    r_encoder_rate = ntproperty('/r_encoder_rate', 0)


    def robotInit(self):
        self.prior_autospeed = 0

        self.joystick = wpilib.Joystick(0)

        self.left_motor_master = CANSparkMax(1, MotorType.kBrushless)
        self.right_motor_master = CANSparkMax(4, MotorType.kBrushless)

        # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(
            self.left_motor_master,
            CANSparkMax(2, MotorType.kBrushless),
            CANSparkMax(3, MotorType.kBrushless)
        )

        self.right_motors = wpilib.SpeedControllerGroup(
            self.right_motor_master,
            CANSparkMax(5, MotorType.kBrushless),
            CANSparkMax(6, MotorType.kBrushless)
        )

        # Configure Gyro

        # Note that the angle from the NavX and all implementors of wpilib Gyro
        # must be negated because getAngle returns a clockwise positive angle
        self.gyro = navx.AHRS.create_spi()

        # Configure drivetrain movement

        self.drive = DifferentialDrive(self.left_motors, self.right_motors)
        self.drive.setDeadband(0)

        # Configure encoder related functions -- getDistance and getrate should return
        # units and units/s
        self.encoder_constant = (1 / self.GEARING) * self.WHEEL_DIAMETER * math.pi

        self.left_encoder = self.left_motor_master.getEncoder()
        self.left_encoder.setPositionConversionFactor(self.encoder_constant)
        self.left_encoder.setVelocityConversionFactor(self.encoder_constant / 60)

        self.right_encoder = self.right_motor_master.getEncoder()
        self.right_encoder.setPositionConversionFactor(self.encoder_constant)
        self.right_encoder.setVelocityConversionFactor(self.encoder_constant / 60)

        self.left_encoder.setPosition(0);
        self.right_encoder.setPosition(0);

        # Set the update rate instead of using flush because of a ntcore bug
        # -> probably don't want to do this on a robot in competition
        NetworkTables.getDefault().setUpdateRate(0.010)

    def disabledInit(self):
        print('Robot disabled')
        self.drive.tankDrive(0, 0)

    def robotPeriodic(self):
        # feedback for users, but not used by the control program
        self.l_encoder_pos = self.left_encoder.getPosition()
        self.l_encoder_rate = self.left_encoder.getVelocity()
        self.r_encoder_pos = self.right_encoder.getPosition()
        self.r_encoder_rate = self.right_encoder.getVelocity()

    def teleopInit(self):
        print('Robot in operator control mode')

    def teleopPeriodic(self):
        self.drive.arcadeDrive(-self.joystick.getY(), self.joystick.getX())
        print(f'Left Distance: {self.left_encoder.getPosition()}')

    def autonomousInit(self):
        print('Robot in autonomous mode')

    # If you wish to just use your own robot program to use with the data logging
    # program, you only need to copy/paste the logic below into your code and
    # ensure it gets called periodically in autonomous mode

    # Additionally, you need to set NetworkTables update rate to 10ms using the
    # setUpdateRate call.

    def autonomousPeriodic(self):
        # Retrieve values to send back before telling the motors to do somethin

        now = wpilib.Timer.getFPGATimestamp()

        leftPosition = self.left_encoder.getPosition()
        leftRate = self.left_encoder.getVelocity()

        rightPosition = self.right_encoder.getPosition()
        rightRate = self.right_encoder.getVelocity()

        battery = wpilib.RobotController.getInputVoltage()
        motorVolts = battery * abs(self.prior_autospeed)

        leftMotorVolts = motorVolts
        rightMotorVolts = motorVolts

        # Retrieve the commanded speed from NetworkTables
        autospeed = self.autoSpeedEntry
        self.prior_autospeed = autospeed

        # command motors to do things
        self.drive.tankDrive((-1 if self.rotateEntry else 1) * autospeed, autospeed, False)

        # send telemetry data array back to NT
        number_array = [
            now, battery, autospeed, leftMotorVolts, rightMotorVolts,
            leftPosition, rightPosition, leftRate, rightRate,
            math.radians(-self.gyro.getAngle())
        ]

        self.telemetry_entry = number_array

if __name__ == '__main__':
    wpilib.run(Robot)