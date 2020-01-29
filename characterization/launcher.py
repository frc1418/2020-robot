
import wpilib
from networktables.util import ntproperty
from networktables import NetworkTablesInstance
from rev import CANSparkMax, MotorType

class Robot(wpilib.TimedRobot):

  GEARING = 1  # Gear ratio

  encoder_pos = ntproperty('/robot/encoder_pos', 0)
  encoder_rate = ntproperty('/robot/encoder_rate', 0)

  autoSpeedEntry = ntproperty('/robot/autospeed', 0, writeDefault=False)
  telemetryEntry = ntproperty('/robot/telemetry', 0, writeDefault=False)

  def createObjects(self):
    self.joystick = wpilib.Joystick(0)

    self.motor = CANSparkMax(7, MotorType.kBrushless)
    self.motor.setInterved(True)

    self.priorAutospeed = 0
    # TODO: Check if we need IdleMode.kBrake
    # self.motor.setIdleMode(IdleMode.kBrake);

    self.encoder = self.motor.getEncoder()

    # //
    # // Configure encoder related functions -- getDistance and getrate should
    # // return units and units/s
    # //
    
    # % if units == 'Degrees':
    # double encoderConstant = (1 / GEARING) * 360
    # % elif units == 'Radians':
    # double encoderConstant = (1 / GEARING) * 2. * Math.PI;
    # % elif units == 'Rotations':
    self.encoderConstant = (1 / GEARING) * 1
    # % endif

    self.encoder.setPosition(0)

    NetworkTableInstance.getDefault().setUpdateRate(0.010)

  def disabledInit(self):
    self.motor.set(0)


  def getEncoderPosition(self):
    return self.encoder.getPosition() * self.encoderConstant

  # In Rotations per second
  def getEncoderRate(self):
    return self.encoder.getRate() * self.encoderConstant / 60

  def robotPeriodic(self):
    self.encoder_pos = self.getEncoderPosition()
    self.encoder_rate = self.getEncoderRate()

  def teleopPeriodic(self):
    self.motor.set(-self.joystick.getY())

  def autonomousPeriodic(self):

    # // Retrieve values to send back before telling the motors to do something
    now = wpilib.Timer.getFPGATimestamp()

    position = self.getEncoderPosition()
    rate = self.getEncoderRate()

    battery = wpilib.RobotController.getBatteryVoltage()

    motorVolts = self.motor.getBusVoltage() * self.motor.getAppliedOutput();

    # // Retrieve the commanded speed from NetworkTables
    autospeed = self.autoSpeedEntry
    self.priorAutospeed = autospeed;

    self.motor.set(autospeed);

    self.telemetryEntry = [now, battery, autospeed, motorVolts, position, rate]
