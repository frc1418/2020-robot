import magicbot
import wpilib
from magicbot import tunable
from rev.color import ColorSensorV3
from robotpy_ext.autonomous import AutonomousModeSelector
from robotpy_ext.control.toggle import Toggle
from wpilib.buttons import JoystickButton
from wpilib.kinematics import DifferentialDriveKinematics
import math

from wpilib import CameraServer
from common.ctre import WPI_TalonSRX, WPI_VictorSPX
from common.limelight import Limelight
from common.navx import navx
from common.rev import CANSparkMax, IdleMode, MotorType, CANEncoder
from components import Align, ControlPanel, Drive, Intake, Launcher, Odometry, Follower
from entry_points.trajectory_generator import KINEMATICS, DRIVE_FEEDFORWARD, load_trajectories
from controllers.panelSpinner import PanelSpinner

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
    # Order of components determines order that execute methods are run
    # State Machines
    panel_spinner: PanelSpinner
    # Other components
    align: Align
    odometry: Odometry
    follower: Follower
    intake: Intake
    control_panel: ControlPanel
    launcher: Launcher
    drive: Drive

    WHEEL_DIAMETER = 0.1524  # Units: Meters
    GEARING = 7.56  # 7.56:1 gear ratio

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
        self.btn_launcher_motor55 = JoystickButton(self.joystick_alt, 10)
        self.btn_slow_movement = Toggle(self.joystick_right, 3)
        self.btn_intake_solenoid = Toggle(self.joystick_alt, 4)
        self.btn_scissor_extend = Toggle(self.joystick_right, 5)
        self.btn_color_sensor = JoystickButton(self.joystick_left, 5)
        self.btn_cp_stop = JoystickButton(self.joystick_left, 2)
        self.btn_invert_y_axis = JoystickButton(self.joystick_left, 1)

        # Set up motors for encoders
        self.master_left = CANSparkMax(1, MotorType.kBrushless)
        self.master_right = CANSparkMax(4, MotorType.kBrushless)
        self.encoder_constant = (1 / self.GEARING) * self.WHEEL_DIAMETER * math.pi

        self.left_encoder = self.master_left.getEncoder()
        self.left_encoder.setPositionConversionFactor(self.encoder_constant)
        self.left_encoder.setVelocityConversionFactor(self.encoder_constant / 60)

        self.right_encoder = self.master_right.getEncoder()
        self.right_encoder.setPositionConversionFactor(self.encoder_constant)
        self.right_encoder.setVelocityConversionFactor(self.encoder_constant / 60)

        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)

        # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(
            self.master_left,
            CANSparkMax(2, MotorType.kBrushless),
            CANSparkMax(3, MotorType.kBrushless)
        )

        self.right_motors = wpilib.SpeedControllerGroup(
            self.master_right,
            CANSparkMax(5, MotorType.kBrushless),
            CANSparkMax(6, MotorType.kBrushless)
        )

        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # Winch
        self.winch_motors = wpilib.SpeedControllerGroup(CANSparkMax(8, MotorType.kBrushless), CANSparkMax(9, MotorType.kBrushless))
        self.scissor_solenoid = wpilib.DoubleSolenoid(6, 7)

        # Control Panel Spinner
        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.cp_solenoid = wpilib.DoubleSolenoid(5, 4)  # Reversed numbers so kForward is extended
        self.cp_motor = CANSparkMax(10, MotorType.kBrushed)
        self.cp_motor.setIdleMode(IdleMode.kBrake)

        # Intake
        self.intake_motors = wpilib.SpeedControllerGroup(WPI_VictorSPX(1), WPI_VictorSPX(4))
        self.intake_solenoid = wpilib.DoubleSolenoid(2, 1)

        # Launcher
        self.launcher_motors = wpilib.SpeedControllerGroup(WPI_VictorSPX(2), WPI_VictorSPX(3))
        self.launcher_solenoid = wpilib.Solenoid(0)

        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()

        # Limelight
        self.limelight = Limelight()

        # Camera Stream
        CameraServer.launch('camera/camera.py:main')

        # Robot motion control
        self.kinematics = KINEMATICS  # Use kinematics from inner trajectory generator code
        self.drive_feedforward = DRIVE_FEEDFORWARD
        self.trajectories = load_trajectories()

    def autonomous(self):
        self.right_motors.setInverted(True)
        super().autonomous()

    def teleopInit(self):
        self.right_motors.setInverted(False)
        self.drive.squared_inputs = True
        self.drive.speed_constant = 1.05
        self.drive.rotational_constant = 0.5
        self.inverse = -1

    def teleopPeriodic(self):
        if self.btn_invert_y_axis.get():
            self.inverse *= -1

        self.drive.move(self.inverse * self.joystick_left.getY(),
                        self.joystick_right.getX())

        # Align (Overrides self.drive.move() because it's placed after)
        if self.btn_align.get() and self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw(), relative=True)

        if self.btn_align.get():
            self.drive.align()
        else:
            self.drive.set_target(None)

        if self.btn_slow_movement:
            # 10% of original values
            self.drive.rotational_constant = 0.08
            self.drive.speed_constant = 0.105

        # Control Panel Spinner
        self.control_panel.set_solenoid(self.btn_cp_extend.get())
        if self.btn_scissor_extend.get():
            self.scissor_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.scissor_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

        # Color Sensor
        if self.btn_color_sensor.get():
            self.panel_spinner.spin_to(position=True)

        if self.btn_cp_motor.get():
            self.control_panel.spin(0.5)

        # Launcher
        if self.btn_launcher_motor.get():
            self.launcher.setPercentOutput(-0.6)
        elif self.btn_launcher_motor70.get():
            self.launcher.setPercentOutput(-0.7)
        elif self.btn_launcher_motor55.get():
            self.launcher.setPercentOutput(-0.55)
        else:
            self.launcher.setPercentOutput(0)

        if self.btn_launcher_solenoid.get():
            self.launcher.fire()

        if self.btn_cp_stop.get():
            self.panel_spinner.done()

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
            self.winch_motors.set(-1)
        else:
            self.winch_motors.set(0)  # Must use set(0) when not pressed because there is no component

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
