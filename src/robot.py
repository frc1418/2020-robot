import magicbot
import navx
import wpilib
from magicbot import tunable
from robotpy_ext.control.toggle import Toggle
from wpilib.buttons import JoystickButton

from common.ctre import WPI_TalonSRX, WPI_VictorSPX
from common.differential import DifferentialDriveKinematics
from common.limelight import Limelight
from common.rev import CANSparkMax, MotorType
from components import Align, ControlPanel, Drive, Intake, Odometry


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
    drive: Drive
    intake: Intake
    control_panel: ControlPanel
    odometry: Odometry
    align: Align
    # TODO: Add launcher component

    TRACK_WIDTH = 0.43  # Units: Meters

    def createObjects(self):
        # Joysticks
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)

        # Buttons
        self.btn_launcher_solenoid = JoystickButton(self.joystick, 1)
        self.btn_align = Toggle(self.joystick_alt, 2)
        self.btn_intake_in = JoystickButton(self.joystick_alt, 3)
        self.btn_intake_out = JoystickButton(self.joystick_alt, 4)
        self.btn_cp_extend = Toggle(self.joystick_alt, 5)
        self.btn_winch = JoystickButton(self.joystick_alt, 6)
        self.btn_cp_motor = JoystickButton(self.joystick_alt, 7)
        self.btn_launcher_motor = Toggle(self.joystick_alt, 12)

        # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(1, MotorType.kBrushless), CANSparkMax(2, MotorType.kBrushless), CANSparkMax(3, MotorType.kBrushless))
        self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(4, MotorType.kBrushless), CANSparkMax(5, MotorType.kBrushless), CANSparkMax(6, MotorType.kBrushless))

        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # Winch
        self.winch_motor = CANSparkMax(8, MotorType.kBrushless)

        # Control Panel Spinner
        self.cp_solenoid = wpilib.DoubleSolenoid(5, 4)  # Reversed numbers so kForward is extended
        self.cp_motor = CANSparkMax(10, MotorType.kBrushed)

        # Intake
        self.intake_motor = WPI_VictorSPX(1)

        # Launcher
        self.launcher_motor = CANSparkMax(7, MotorType.kBrushed)
        self.launcher_motor.restoreFactoryDefaults()
        self.launcher_motor.setOpenLoopRampRate(0)

        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()

        # Kinematics
        self.kinematics = DifferentialDriveKinematics(self.TRACK_WIDTH)  # Track width in meters

        # Limelight
        self.limelight = Limelight()

    def teleopInit(self):
        self.drive.squared_inputs = True
        self.drive.speed_constant = 1.05
        self.drive.rotational_constant = 0.5

    def teleopPeriodic(self):
        self.drive.move(-self.joystick_left.getY(),
                        self.joystick_right.getX())
       
        # Align (Overrides self.drive.move() because it's placed after)
        if self.btn_align.get():
            self.align.align(self.limelight.getYaw())

        # Control Panel Spinner
        self.control_panel.set_solenoid(self.btn_cp_extend.get())

        # Launcher
        if self.btn_launcher_motor.get():
            self.launcher_motor.set(0.75)
        else:
            self.launcher_motor.set(0)

        # Intake
        if self.btn_intake_out.get():
            self.intake.spin(1)
        elif self.btn_intake_in.get():
            self.intake.spin(-1)

        # Winch
        if self.winch_button.get():
            self.winch_motor.set(1)
        else:
            self.winch_motor.set(0)  # Must use set(0) when not pressed because there is no component


if __name__ == '__main__':
    wpilib.run(Robot)
