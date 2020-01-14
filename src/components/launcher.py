from rev import CANSparkMax, ControlType

class Launcher:
    launcher_motor: CANSparkMax

    def setup(self):
        self.PID_Controller = self.launcher_motor.getPIDController()
        self.encoder = self.launcher_motor.getEncoder()

    def warmUp(self):
        self.warm_Up = False
        self.launcher_motor.set(1)
        if (self.launcher_motor.getVoltage() > 8):
            self.warm_Up = True

    def execute(self):
        if (self.warm_Up == True):
            self.PID_Controller.setReference(1000, ControlType.kVelocity, pidSlot=0, arbFeedforward=0)
            if (self.encoder.getVelocity() > 990):
                self.readyToFire = True
                 
            