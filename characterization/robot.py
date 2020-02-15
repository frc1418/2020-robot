
import wpilib
from networktables.util import ntproperty
from networktables import NetworkTablesInstance
import math
# Add VictorSPX try import
try:
    from ctre import WPI_VictorSPX
except ImportError:
    WPI_VictorSPX = type('WPI_VictorSPX', (wpilib.interfaces.SpeedController,), {})

class Robot(wpilib.TimedRobot):

    ENCODER_PULSES_PER_REV = 1024  # Ignore quadrature decoding (4x)
    GEARING = (3.25 / 1)  # 3.25:1 in gearbox

    encoder_pos = ntproperty('/robot/encoder_pos', 0)
    encoder_rate = ntproperty('/robot/encoder_rate', 0)

    autoSpeedEntry = ntproperty('/robot/autospeed', 0, writeDefault=False)
    telemetryEntry = ntproperty('/robot/telemetry', [0, 0, 0, 0, 0, 0], writeDefault=False)

    def robotInit(self):
        self.joystick = wpilib.Joystick(0)

        self.motors = wpilib.SpeedControllerGroup(WPI_VictorSPX(2), WPI_VictorSPX(3))
        self.motors.setInverted(True)

        self.priorAutospeed = 0
        # TODO: Check if we need IdleMode.kBrake
        # self.motor.setIdleMode(IdleMode.kBrake);

        self.encoder = wpilib.Encoder(1, 2)

        # //
        # // Configure encoder related functions -- getDistance and getrate should
        # // return units and units/s
        # //

        # % if units == 'Degrees':
        # double encoderConstant = (1 / GEARING) * 360
        # % elif units == 'Radians':
        # double encoderConstant = (1 / GEARING) * 2. * Math.PI;
        # % elif units == 'Rotations':
        self.encoderConstant = (1 / (self.ENCODER_PULSES_PER_REV * self.GEARING))
        self.encoder.setDistancePerPulse(self.encoderConstant)

        self.encoder.reset()

        NetworkTablesInstance.getDefault().setUpdateRate(0.010)

    def disabledInit(self):
        self.motors.set(0)

    def robotPeriodic(self):
        self.encoder_pos = self.encoder.getDistance()
        self.encoder_rate = self.encoder.getRate()

    def teleopPeriodic(self):
        self.motors.set(-self.joystick.getY())
        print(self.encoder.get())

    def autonomousPeriodic(self):

        # // Retrieve values to send back before telling the motors to do something
        now = wpilib.Timer.getFPGATimestamp()

        position = self.encoder.getDistance()
        rate = self.encoder.getRate()

        battery = wpilib.RobotController.getInputVoltage()

        motorVolts = battery * abs(self.priorAutospeed)

        # // Retrieve the commanded speed from NetworkTables
        autospeed = self.autoSpeedEntry
        self.priorAutospeed = autospeed

        self.motors.set(autospeed)

        self.telemetryEntry = [
            now, battery, autospeed,
            motorVolts, position, rate
        ]

if __name__ == '__main__':
    wpilib.run(Robot)