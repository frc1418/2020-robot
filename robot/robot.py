import magicbot
import wpilib
import wpilib.drive
from wpilib.buttons import JoystickButton
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from robot import drive
from robotpy_ext.common_drivers import navx
from magicbot import tunable
from rev import CANSparkMax, MotorType

class Robot(magicbot.MagicRobot):
    drive: drive.Drive
    time = tunable(0)
    voltage = tunable(0)
    rotation = tunable(0)
    color = tunable('')
    getColor = True

    def createObects(self):
         # Joysticks
        self.joystick_left = wpilib.Joystick(0)
        self.joystick_right = wpilib.Joystick(1)
        self.joystick_alt = wpilib.Joystick(2)
       
       # TODO: decide what buttons should do. Need to talk to drivers

        # Set up Speed Controller Groups
        self.left_motors = wpilib.SpeedControllerGroup(CANSparkMax(10, MotorType.kBrushless), CANSparkMax(20, MotorType.kBrushless), CANSparkMax(30, MotorType.kBrushless))
        self.right_motors = wpilib.SpeedControllerGroup(CANSparkMax(40, MotorType.kBrushless), CANSparkMax(50, MotorType.kBrushless), CANSparkMax(60, MotorType.kBrushless))

        # Drivetrain
        self.train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()

        #Utility
        self.ds = wpilib.DriverStation.getInstance()
        self.timer = wpilib.Timer()
        self.pdp = wpilib.PowerDistributionPanel(0)
    
    def robotPeriodic(self):
        self.time = int(self.timer.getMatchTime())
        self.voltage = self.pdp.getVoltage()
        self.rotation = self.navx.getAngle() % 360
    
    def autonomous(self):
        pass

    def disabledInit(self):
        self.navx.reset()
    
    def disabledPeriodic(self):
        pass

    def teleopInit(self):
        self.drive.squared_inputs = True
        self.drive.rotational_constant = 0.5
        drive.train.setDeadband(0.1)

    def teleopPeriodic(self):
        self.drive.move(-self.joystick_left.getY(),
                        self.joystick_right.getX())
        
        # Checks whether or not the FMS has sent what color the color wheel needs to be spun to
        if len(self.ds.getGameSpecificMessage()) > 0 and self.getColor == True:
            self.color = self.ds.getGameSpecificMessage()
            if self.color == 'R':
                # red color
                pass
            elif self.color == 'G':
                # green color
                pass
            elif self.color == 'B':
                # blue color
                pass
            elif self.color == 'Y':
                # yellow color
                pass
            self.getColor = False

if __name__ == '__main__':
    wpilib.run(Robot)







