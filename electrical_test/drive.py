import magicbot
import wpilib
import wpilib.drive
import math
import time
import random
from magicbot import tunable
from wpilib.buttons import JoystickButton
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from networktables import NetworkTables, NetworkTablesInstance
from networktables.util import ntproperty

from robotpy_ext.control.toggle import Toggle

from robotpy_ext.misc.periodic_filter import PeriodicFilter, logging

from ctre import WPI_VictorSPX
from rev import CANSparkMax, MotorType


class TestRobot(magicbot.MagicRobot):
    shooter_speed = tunable(0, writeDefault=False)
    time = tunable(0)
    voltage = tunable(0)
    rotation = tunable(0)
    
    def createObjects(self):
        self.launcher_motor = CANSparkMax(7, MotorType.kBrushed)
        self.launcher_motor.restoreFactoryDefaults()
        self.launcher_motor.setOpenLoopRampRate(0)
        self.intake = WPI_VictorSPX(1)
        self.solenoid = wpilib.Solenoid(0)
        self.encoder = self.launcher_motor.getEncoder()
        # self.shooter_running = False
        self.logger.addFilter(PeriodicFilter(1, bypass_level=logging.WARN))

        # 2000rpm is a good setting

        # set up joystick buttons
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)

        # self.left_button = Toggle(self.joystick_alt, 10)
        # self.middle_button = Toggle(self.joystick_alt, 11)
        # self.right_button = Toggle(self.joystick_alt, 12)
        self.one = JoystickButton(self.joystick_alt, 7)
        self.two = JoystickButton(self.joystick_alt, 8)
        self.three = JoystickButton(self.joystick_alt, 9)
        self.four = JoystickButton(self.joystick_alt, 10)
        self.five = JoystickButton(self.joystick_alt, 11)
        self.six = JoystickButton(self.joystick_alt, 12)

        self.intake_in = JoystickButton(self.joystick_alt, 3)
        self.intake_out = JoystickButton(self.joystick_alt, 4)
        self.btn_solenoid = JoystickButton(self.joystick_alt, 1)

         # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(1, MotorType.kBrushless), CANSparkMax(2, MotorType.kBrushless), CANSparkMax(3, MotorType.kBrushless))
        self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(4, MotorType.kBrushless), CANSparkMax(5, MotorType.kBrushless), CANSparkMax(6, MotorType.kBrushless))

        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # self.compressor = wpilib.Compressor()

    def disabledInit(self):
        self.launcher_motor.set(0)
        self.intake.set(0)
        self.solenoid.set(False)

    def teleopInit(self):
        self.train.setDeadband(0.1)
        # self.compressor.start()

    def teleopPeriodic(self):
        self.logger.info(self.encoder.getVelocity())
        self.shooter_speed = self.encoder.getVelocity()

        # if self.left_button.get():
        #     self.launcher_motor.set(-0.2)
        # elif self.middle_button.get():
        #     self.launcher_motor.set(-0.5)
        # elif self.right_button.get():
        #     self.launcher_motor.set(-0.7)
        # else:
        #     self.launcher_motor.set(0)


 
        if self.intake_in.get():
            self.intake.set(-1)
        elif self.intake_out.get():
            self.intake.set(1)
        else:
            self.intake.set(0)

        # BUTTON SETTINGS
        if self.one.get():
            self.launcher_motor.set(0)
        elif self.two.get():
            self.launcher_motor.set(-0.2)
        elif self.three.get():
            self.launcher_motor.set(-0.4)
        elif self.four.get():
            self.launcher_motor.set(-0.6)
        elif self.five.get():
            self.launcher_motor.set(-0.8)
        elif self.six.get():
            self.launcher_motor.set(-1)
        else:
            # self.launcher_motor.set(-self.joystick_alt.getAxis(3))
            self.launcher_motor.set(0)

        # SLIDER SETTINGS
        # self.launcher_motor.set(-self.joystick_alt.getAxis(3))
        
        self.solenoid.set(self.btn_solenoid.get())

        self.train.arcadeDrive(-self.joystick_left.getY(),
                        self.joystick_right.getX())


if __name__ == '__main__':
    wpilib.run(TestRobot)