import wpilib
from magicbot import MagicRobot
from rev import CANSparkMax, MotorType
from wpilib.buttons import JoystickButton
from ctre import WPI_VictorSPX

class Robot(MagicRobot):
    def createObjects(self):
        self.joystick = wpilib.Joystick(2)
        self.winch_button = JoystickButton(self.joystick, 6)
        self.cp_extend_button = JoystickButton(self.joystick, 5)
        self.cp_solenoid = wpilib.DoubleSolenoid(4, 5)
        self.cp_motor = CANSparkMax(10, MotorType.kBrushed)

        self.solenoid = wpilib.Solenoid(0)
        self.btn_solenoid = JoystickButton(self.joystick, 1)

        self.intake = WPI_VictorSPX(1)

        self.cp_motor_button = JoystickButton(self.joystick, 7)
        self.winch_motor = CANSparkMax(8, MotorType.kBrushless)

        self.six = JoystickButton(self.joystick, 12)
        self.shooter_motor = CANSparkMax(7, MotorType.kBrushed)
        self.shooter_motor.restoreFactoryDefaults()
        self.shooter_motor.setOpenLoopRampRate(0)

        self.intake_in = JoystickButton(self.joystick, 3)
        self.intake_out = JoystickButton(self.joystick, 4)

    def teleopPeriodic(self):
        if self.winch_button.get():
            self.winch_motor.set(1)
        else:
            self.winch_motor.set(0)

        if self.six.get():
            # 75% is good for initiation line

            self.shooter_motor.set(0.75)
        else:
            self.shooter_motor.set(0)

        self.solenoid.set(self.btn_solenoid.get())

        if self.intake_in.get():
            self.intake.set(-1)
        elif self.intake_out.get():
            self.intake.set(1)
        else:
            self.intake.set(0)

        if self.cp_motor_button.get():
            self.cp_motor.set(0.7)
        else:
            self.cp_motor.set(0)
        
        if self.cp_extend_button.get() and self.cp_solenoid.get() != wpilib.DoubleSolenoid.Value.kReverse:
            self.cp_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
        if not  self.cp_extend_button.get() and self.cp_solenoid.get() != wpilib.DoubleSolenoid.Value.kForward:
            self.cp_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

if __name__ == '__main__':
    wpilib.run(Robot)