from common.rev import CANSparkMax, ControlType
from magicbot import will_reset_to
import wpilib


class Launcher:
    launcher_motors: wpilib.SpeedControllerGroup
    launcher_solenoid: wpilib.Solenoid
    speed = will_reset_to(0)
    decimal = will_reset_to(0)
    control_velocity = will_reset_to(False)
    shoot = will_reset_to(False)

    def setup(self):
        # self.PID_Controller = self.launcher_motor.getPIDController()
        # self.encoder = self.launcher_motor.getEncoder()
        pass

    def setVelocity(self, speed):
        self.speed = speed
        self.control_velocity = True
        # TODO: make dictionary(?) that lets us find speed of motor depending on distance

    def setPercentOutput(self, decimal):
        self.decimal = decimal

    # def getSpeed(self):
        # return self.encoder.getVelocity()

    def fire(self):
        self.shoot = True

    def execute(self):
        if self.control_velocity:
            pass
            # self.PID_Controller.setReference(self.speed, ControlType.kVelocity, pidSlot=0, arbFeedforward=0)
        else:
            self.launcher_motors.set(self.decimal)
        self.launcher_motors.set(self.decimal)
        self.launcher_solenoid.set(self.shoot)
