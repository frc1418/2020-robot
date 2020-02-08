import math

import magicbot
import navx
import wpilib
from wpilib.drive import DifferentialDrive
from ctre import WPI_TalonSRX
from networktables.networktables import NetworkTables, NetworkTablesInstance
from networktables.util import ntproperty
from magicbot import tunable


class Robot(magicbot.MagicRobot):
    WHEEL_DIAMETER = 0.1524  # Units: Meters
    ENCODER_PULSE_PER_REV = 1024

    autoSpeedEntry = ntproperty('/robot/autospeed', 0.0)
    telemetry_entry = ntproperty('/robot/telemetry', [0.0], writeDefault=False)
    rotateEntry = ntproperty('/robot/rotate', False)

    l_encoder_pos = tunable(0)
    l_encoder_rate = tunable(0)
    r_encoder_pos = tunable(0)
    r_encoder_rate = tunable(0)


    def createObjects(self):
        self.prior_autospeed = 0

        self.joystick = wpilib.Joystick(0)

        self.left_motors = wpilib.SpeedControllerGroup(WPI_TalonSRX(10), WPI_TalonSRX(30))
        self.right_motors = wpilib.SpeedControllerGroup(WPI_TalonSRX(20), WPI_TalonSRX(40))

        # Configure Gyro

        # Note that the angle from the NavX and all implementors of wpilib Gyro
        # must be negated because getAngle returns a clockwise positive angle
        self.gyro = navx.AHRS.create_spi()

        # Configure drivetrain movement

        self.drive = DifferentialDrive(self.left_motors, self.right_motors)
        self.drive.setDeadband(0)

        # Configure encoder related functions -- getDistance and getrate should return
        # units and units/s

        self.encoder_constant = (1 / self.ENCODER_PULSE_PER_REV) * self.WHEEL_DIAMETER * math.pi

        self.left_encoder = wpilib.Encoder(0, 1)
        self.left_encoder.setDistancePerPulse(self.encoder_constant)

        self.right_encoder = wpilib.Encoder(2, 3)
        self.right_encoder.setReverseDirection(True)
        self.right_encoder.setDistancePerPulse(self.encoder_constant)

        # Set the update rate instead of using flush because of a ntcore bug
        # -> probably don't want to do this on a robot in competition
        NetworkTables.getDefault().setUpdateRate(0.010)

    def disabledInit(self):
        print('Robot disabled')
        self.drive.tankDrive(0, 0)

    def robotPeriodic(self):
        # feedback for users, but not used by the control program
        self.l_encoder_pos = self.left_encoder.get()
        self.l_encoder_rate = self.left_encoder.getRate()
        self.r_encoder_pos = self.right_encoder.get()
        self.r_encoder_rate = self.right_encoder.getRate()

    def teleopInit(self):
        print('Robot in operator control mode')

    def teleopPeriodic(self):
        self.drive.arcadeDrive(-self.joystick.getY(), self.joystick.getX())

    def autonomousInit(self):
        print('Robot in autonomous mode')

    # If you wish to just use your own robot program to use with the data logging
    # program, you only need to copy/paste the logic below into your code and
    # ensure it gets called periodically in autonomous mode

    # Additionally, you need to set NetworkTables update rate to 10ms using the
    # setUpdateRate call.

    def autonomousPeriodic(self):
        # Retrieve values to send back before telling the motors to do something
        now = wpilib.Timer.getFPGATimestamp()

        leftPosition = self.left_encoder.getDistance()
        leftRate = self.left_encoder.getRate()

        rightPosition = self.right_encoder.getDistance()
        rightRate = self.right_encoder.getRate()

        battery = wpilib.RobotController.getBatteryVoltage()
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
