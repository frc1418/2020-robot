import magicbot
import wpilib
from magicbot import tunable
from rev.color import ColorSensorV3
from robotpy_ext.autonomous import AutonomousModeSelector
from robotpy_ext.control.toggle import Toggle
from wpilib.buttons import JoystickButton

from common.ctre import WPI_TalonSRX, WPI_VictorSPX
from wpilib.kinematics import DifferentialDriveKinematics
from common.limelight import Limelight
from common.navx import navx
from common.rev import CANSparkMax, IdleMode, MotorType
from components import Align, ControlPanel, Drive, Intake, Launcher, Odometry
from common.camera_server import CameraServer

r"""
/ \                / \
\ /       (+)      \ /
           |
           |X
(+) -------|--Y----  (-)
           |
           |
/ \       (-)      / \
\ /                \ /

Counter-Clockwise is Positive
   /-\ ^
   |X| | (+)
   \-/ |
   -->
"""


class Robot(magicbot.MagicRobot):
    extunable = tunable(5)
    drive: Drive
    intake: Intake
    control_panel: ControlPanel
    odometry: Odometry
    align: Align
    launcher: Launcher

    TRACK_WIDTH = 0.43  # Units: Meters

    def createObjects(self):
        # Joysticks
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)

        # Buttons
        self.btn_launcher_solenoid = JoystickButton(self.joystick_alt, 1)
        self.btn_align = JoystickButton(self.joystick_alt, 2)
        self.btn_intake_in = JoystickButton(self.joystick_alt, 3)
        self.btn_intake_out = JoystickButton(self.joystick_alt, 5)
        self.btn_cp_extend = Toggle(self.joystick_left, 4)
        self.btn_winch = JoystickButton(self.joystick_alt, 6)
        self.btn_cp_motor = JoystickButton(self.joystick_left, 3)
        self.btn_launcher_motor = JoystickButton(self.joystick_alt, 12)
        self.btn_launcher_motor70 = JoystickButton(self.joystick_alt, 11)
        self.btn_slow_movement = Toggle(self.joystick_right, 3)
        self.btn_intake_solenoid = Toggle(self.joystick_alt, 4)
        self.btn_scissor_extend = Toggle(self.joystick_right, 5)
        self.btn_color_sensor = JoystickButton(self.joystick_left, 5)
        self.btn_cp_stop = JoystickButton(self.joystick_left, 2)

        # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(
            CANSparkMax(1, MotorType.kBrushless),
            CANSparkMax(2, MotorType.kBrushless),
            CANSparkMax(3, MotorType.kBrushless)
        )

        self.right_motors = wpilib.SpeedControllerGroup(
            CANSparkMax(4, MotorType.kBrushless),
            CANSparkMax(5, MotorType.kBrushless),
            CANSparkMax(6, MotorType.kBrushless)
        )

        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # Winch
        self.winch_motor = CANSparkMax(8, MotorType.kBrushless)
        self.scissor_solenoid = wpilib.DoubleSolenoid(6, 7)

        # Control Panel Spinner
        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.cp_solenoid = wpilib.DoubleSolenoid(5, 4)  # Reversed numbers so kForward is extended
        self.cp_motor = CANSparkMax(10, MotorType.kBrushed)
        self.cp_motor.setIdleMode(IdleMode.kBrake)

        # Intake
        self.intake_motor = WPI_VictorSPX(1)
        self.intake_solenoid = wpilib.DoubleSolenoid(2, 1)

        # Launcher
        self.launcher_motors = wpilib.SpeedControllerGroup(WPI_VictorSPX(2), WPI_VictorSPX(3))
        self.launcher_solenoid = wpilib.Solenoid(0)

        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()

        # Limelight
        self.limelight = Limelight()

        # Kinematics
        self.kinematics = DifferentialDriveKinematics(self.TRACK_WIDTH)  # Track width in meters

        # Camera Stream
        CameraServer.launch('camera/camera.py:main')

    def teleopInit(self):
        self.drive.squared_inputs = True
        self.drive.speed_constant = 1.05
        self.drive.rotational_constant = 0.5

    def teleopPeriodic(self):
        self.drive.move(-self.joystick_left.getY(),
                        self.joystick_right.getX())

        # Align (Overrides self.drive.move() because it's placed after)
        if self.btn_align.get() and True:
            self.drive.align(self.limelight.getYaw(), relative=True)

        if self.btn_slow_movement:
            # 10% of original values
            self.drive.rotational_constant = 0.05
            self.drive.speed_constant = 0.105
            self.drive.deadband = 0.05
        else:
            self.drive.rotational_constant = 0.5
            self.drive.speed_constant = 1.05

        # Control Panel Spinner
        self.control_panel.set_solenoid(self.btn_cp_extend.get())
        if self.btn_scissor_extend.get():
            self.scissor_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.scissor_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

        # Color Sensor
        if self.btn_color_sensor.get():
            self.control_panel.spin_to(position=True)

        if self.btn_cp_motor.get():
            self.control_panel.spin(0.5)

        # Launcher
        if self.btn_launcher_motor.get():
            self.launcher.setPercentOutput(-0.6)
        elif self.btn_launcher_motor70.get():
            self.launcher.setPercentOutput(-0.7)
        else:
            self.launcher.setPercentOutput(0)

        if self.btn_launcher_solenoid.get():
            self.launcher.fire()

        if self.btn_cp_stop.get():
            self.control_panel.done()

        # Intake
        if self.btn_intake_out.get():
            self.intake.spin(1)
        elif self.btn_intake_in.get():
            self.intake.spin(-1)

        if self.btn_intake_solenoid.get():
            self.intake_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
        else:
            self.intake_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

        # Winch
        if self.btn_winch.get():
            self.winch_motor.set(-1)
        else:
            self.winch_motor.set(0)  # Must use set(0) when not pressed because there is no component

        # slow movement using POV on joystick_alt
        # if self.joystick_alt.getPOV() == 0:
        #     self.drive.move(0.2, 0)
        # elif self.joystick_alt.getPOV() == 180:
        #     self.drive.move(-0.2, 0)
        # elif self.joystick_akt.getPOV() == 90:
        #     self.drive.move(0, -0.2)
        # elif self.joystick_alt.getPOV() == 270:
        #     self.drive.move(0, 0.2)

        # print(self.joystick_alt.getPOV())


if __name__ == '__main__':
    wpilib.run(Robot)
