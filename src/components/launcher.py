from common.rev import CANSparkMax, ControlType
from magicbot import will_reset_to
import wpilib

class Launcher:
    launcher_motor: CANSparkMax
    launcher_solenoid: wpilib.Solenoid
    speed = will_reset_to(0)
    decimal = will_reset_to(0)
    control_velocity = will_reset_to(True)

    def setup(self):
        self.PID_Controller = self.launcher_motor.getPIDController()
        self.encoder = self.launcher_motor.getEncoder()
        

    def setVelocity(self, speed):
        self.speed = speed
        self.control_velocity = True
        # TODO: make dictionary(?) that lets us find speed of motor depending on distance

    def setPercentOutput(self, decimal):
        self.decimal = decimal
        self.control_velocity = False


    def returnSpeed(self):
        return self.encoder.getVelocity()

    def spin(self):
        if self.control_velocity:
            self.PID_Controller.setReference(self.speed, ControlType.kVelocity, pidSlot=0, arbFeedforward=0)
        else:
            self.launcher_motor.set(self.decimal)

    def fire(self):
        self.launcher_solenoid.set(True)