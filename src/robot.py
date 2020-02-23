import magicbot
import wpilib
from magicbot import tunable
from networktables.util import ntproperty
from rev.color import ColorSensorV3
from robotpy_ext.autonomous import AutonomousModeSelector
from robotpy_ext.control.toggle import Toggle
from wpilib.buttons import JoystickButton
from wpilib.interfaces import GenericHID
from wpilib.kinematics import DifferentialDriveKinematics
import math

from wpilib import CameraServer, Ultrasonic
from common.ctre import WPI_TalonSRX, WPI_VictorSPX
from common.limelight import Limelight
from common.navx import navx
from common.rev import CANSparkMax, IdleMode, MotorType, CANEncoder
from components import Align, ControlPanel, Drive, Intake, Launcher, Odometry, Follower
from entry_points.trajectory_generator import KINEMATICS, DRIVE_FEEDFORWARD, load_trajectories
from controllers.panelSpinner import PanelSpinner
from controllers.auto_launcher import AutoShoot
from common.blinkinLED import BlinkinLED
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
    auto_launcher: AutoShoot
    # Other components
    align: Align
    odometry: Odometry
    follower: Follower
    intake: Intake
    control_panel: ControlPanel
    launcher: Launcher
    drive: Drive

    flipped = tunable(False)
    ntSolenoid_state = ntproperty("/robot/ntSolenoid_state", 0)
    ds_velocity_setpoint = ntproperty('/components/launcher/DS_VEL_SETPOINT', 2100)

    WHEEL_DIAMETER = 0.1524  # Units: Meters
    ENCODER_PULSES_PER_REV = 1024  # Ignore quadrature decoding (4x)
    LAUNCHER_GEARING = (3.25 / 1)  # 3.25:1 in gearbox
    GEARING = 7.56  # 7.56:1 gear ratio

    def createObjects(self):
        # Joysticks
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)

        # Buttons
        self.btn_launcher_solenoid = JoystickButton(self.joystick_alt, 1)
        self.btn_align = JoystickButton(self.joystick_left, 1)
        self.btn_intake_in = JoystickButton(self.joystick_alt, 3)
        self.btn_intake_out = JoystickButton(self.joystick_alt, 4)
        self.btn_cp_extend = Toggle(self.joystick_left, 4)
        self.btn_winch = JoystickButton(self.joystick_alt, 8)
        self.btn_cp_motor = JoystickButton(self.joystick_left, 3)
        self.btn_launcher_motor = JoystickButton(self.joystick_alt, 12)
        self.btn_launcher_motor70 = JoystickButton(self.joystick_alt, 11)
        self.btn_launcher_motor_dynamic = JoystickButton(self.joystick_alt, 9)
        self.btn_launcher_resting = Toggle(self.joystick_alt, 10)
        self.btn_slow_movement = JoystickButton(self.joystick_right, 1)
        self.btn_intake_solenoid = Toggle(self.joystick_right, 5)
        self.btn_scissor_extend = Toggle(self.joystick_alt, 7)
        self.btn_color_sensor = JoystickButton(self.joystick_left, 5)
        self.btn_cp_stop = JoystickButton(self.joystick_left, 2)
        self.btn_invert_y_axis = JoystickButton(self.joystick_left, 6)
        self.btn_rotation_sensitivity = JoystickButton(self.joystick_right, 1)

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
        self.hook_motor = CANSparkMax(12, MotorType.kBrushed)

        # Control Panel Spinner
        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.cp_solenoid = wpilib.DoubleSolenoid(5, 4)  # Reversed numbers so kForward is extended
        self.cp_motor = CANSparkMax(10, MotorType.kBrushed)
        self.cp_motor.setIdleMode(IdleMode.kBrake)
        self.ultrasonic = Ultrasonic(4, 5)
        self.ultrasonic.setAutomaticMode(True)
        self.ultrasonic.setEnabled(True)

        # Intake
        self.intake_motor = WPI_VictorSPX(1)
        self.intake_wheels = WPI_VictorSPX(4)
        self.intake_solenoid = wpilib.DoubleSolenoid(2, 1)
        self.intake_switch = wpilib.DigitalInput(3)

        # Launcher
        # self.launcher_spark = CANSparkMax(40, MotorType.kBrushed)
        # self.launcher_spark.setInverted(True)
        self.launcher_motors = wpilib.SpeedControllerGroup(WPI_VictorSPX(2), WPI_VictorSPX(3))
        self.launcher_solenoid = wpilib.Solenoid(0)
        self.launcher_encoder = wpilib.Encoder(1, 2, True)
        self.encoderConstant = (1 / (self.ENCODER_PULSES_PER_REV * self.LAUNCHER_GEARING))
        self.launcher_encoder.setDistancePerPulse(self.encoderConstant)
        self.launcher_sensor = wpilib.Ultrasonic(6, 7)
        self.launcher_sensor.setAutomaticMode(True)
        self.launcher_sensor.setEnabled(True)

        self.launcher_encoder.reset()

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

        # LED strip
        self.led_driver = BlinkinLED(0)

    def autonomous(self):
        self.right_motors.setInverted(True)
        super().autonomous()
        self.limelight.changePipeline(0)

    def disabledInit(self):
        self.navx.reset()

    def disabledPeriodic(self):
        self.limelight.averagePose()

    def teleopInit(self):
        self.right_motors.setInverted(False)
        self.drive.squared_inputs = True
        self.drive.speed_constant = 1.05
        self.limelight.changePipeline(0)
        self.drive.rotational_constant = 0.5
        self.inverse = 1

    def teleopPeriodic(self):
        # if self.btn_invert_y_axis.get():
        #     self.flipped = True
        #     self.inverse *= -1
        # else:
        #     self.flipped = False

        if self.btn_invert_y_axis.get():
            self.inverse = 1
        else:
            self.inverse = -1
        if self.btn_rotation_sensitivity.get():
            self.drive.rotational_constant = 0.1

        self.drive.move(self.inverse * self.joystick_left.getY(),
                        self.joystick_right.getX())

        # Align (Overrides self.drive.move() because it's placed after)
        if self.btn_align.get() and self.limelight.targetExists():
            self.drive.set_target(self.limelight.getYaw(), relative=True)

        if self.btn_align.get():
            self.limelight.TurnLightOn(True)
            self.drive.align()
        else:
            self.limelight.TurnLightOn(False)
            self.drive.set_target(None)

        if self.btn_slow_movement.get():
            # 10% of original values
            self.drive.rotational_constant = 0.08
            self.drive.speed_constant = 0.105

        # Control Panel Spinner
        self.control_panel.set_solenoid(self.btn_cp_extend.get())
        if self.btn_cp_extend.get():
            self.ntSolenoid_state = True
        else:
            self.ntSolenoid_state = False
        if self.btn_scissor_extend.get():
            self.scissor_solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.scissor_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

        # Color Sensor
        if self.btn_color_sensor.get():
            self.panel_spinner.spin_to()

        if self.btn_cp_motor.get():
            self.control_panel.spin(0.5)

        # Launcher
        if self.btn_launcher_resting.get():
            self.launcher.setVelocity(1000)

        if self.btn_launcher_motor.get():
            self.launcher.setVelocity(2100)
        elif self.btn_launcher_motor70.get():
            self.launcher.setVelocity(1900)
        elif self.btn_launcher_motor_dynamic.get():
            self.launcher.setVelocity(self.ds_velocity_setpoint)

        if self.btn_launcher_solenoid.get():
            self.auto_launcher.fire_when_ready()
            if not self.btn_intake_in.get():
                self.intake.spin(-1)

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
            self.winch_motors.set(0.8)
        else:
            self.winch_motors.set(0)  # Must use set(0) when not pressed because there is no component

        # Hook
        if self.joystick_alt.getPOV(0) == 90:
            self.hook_motor.set(0.5)
        elif self.joystick_alt.getPOV(0) == 270:
            self.hook_motor.set(-0.5)
        else:
            self.hook_motor.set(0)
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
