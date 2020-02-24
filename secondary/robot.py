import magicbot
import wpilib
from magicbot import tunable
from rev import CANSparkMax, MotorType
import navx
from components.drive import Drive


class Robot(magicbot.MagicRobot):
    drive: Drive
    time = tunable(0)
    voltage = tunable(0)
    rotation = tunable(0)

    def createObjects(self):
        # Joysticks
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)

        # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(1, MotorType.kBrushless), CANSparkMax(2, MotorType.kBrushless), CANSparkMax(3, MotorType.kBrushless))
        self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(4, MotorType.kBrushless), CANSparkMax(5, MotorType.kBrushless), CANSparkMax(6, MotorType.kBrushless))

        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()

    def teleopInit(self):
        self.drive.squared_inputs = True
        self.drive.rotational_constant = 0.5
        self.train.setDeadband(0.1)

    def teleopPeriodic(self):
        self.drive.move(-self.joystick_left.getY(),
                        self.joystick_right.getX())


if __name__ == '__main__':
    wpilib.run(Robot)
