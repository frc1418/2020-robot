import magicbot
import wpilib
from common.differential import DifferentialDriveKinematics
from magicbot import tunable
from rev import CANSparkMax, MotorType
import navx
from common.limelight import Limelight
from robotpy_ext.control.toggle import Toggle
from components.drive import Drive
from components.align import Align
from components.controlpanel import ControlPanel


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
    align: Align

    TRACK_WIDTH = 0.43  # Units: Meters

    control_panel: ControlPanel

    def createObjects(self):
        # Joysticks
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)

        # TODO: decide what buttons should do. Need to talk to drivers
        self.align_button = Toggle(self.joystick_alt, 2)
        # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(10, MotorType.kBrushless), CANSparkMax(20, MotorType.kBrushless), CANSparkMax(30, MotorType.kBrushless))
        self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(40, MotorType.kBrushless), CANSparkMax(50, MotorType.kBrushless), CANSparkMax(60, MotorType.kBrushless))

        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()

        # Kinematics
        self.kinematics = DifferentialDriveKinematics(self.TRACK_WIDTH)  # Track width in meters

        #Control Panel Component
        self.cp_motor = CANSparkMax(10, MotorType.kBrushless)

        # Limelight
        self.limelight = Limelight()

    def teleopInit(self):
        self.drive.squared_inputs = True
        self.drive.rotational_constant = 0.5
        self.train.setDeadband(0.1)

    def teleopPeriodic(self):
        self.drive.move(-self.joystick_left.getY(),
                        self.joystick_right.getX()) 
       
        if self.align_button.get():
            self.align.align(self.limelight.getYaw())


if __name__ == '__main__':
    wpilib.run(Robot)
