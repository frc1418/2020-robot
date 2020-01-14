from rev import CANSparkMax, ControlType

class Launcher:
    launcher_motor: CANSparkMax

    def setup(self):
        self.PID_Controller = self.launcher_motor.getPIDController()

    def warmUp(self):
        self.launcher_motor.set(1)
        if (self.launcher_motor.getVoltage() > 8):
            self.readyToFire = True

    def execute(self):
        if (self.readyToFire == True):
            self.PID_Controller.setReference(1000, ControlType.kVelocity, pidSlot=0, arbFeedforward=0)