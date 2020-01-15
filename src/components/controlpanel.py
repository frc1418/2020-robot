import wpilib
import wpilib.drive
from rev import CANSparkMax, MotorType
from magicbot import will_reset_to
from magicbot import tunable
from electrical_test.ColorSensorV3 import ColorSensorV3

class Colors(Enum):
    Blue = ColorSensorV3.RawColor(
        0.187, 0.457, 0.376, 0)  # accurate from ~16.2 cm
    Green = ColorSensorV3.RawColor(
        0.178, 0.573, 0.252, 0)  # accurate from ~12.7 cm
    Red = ColorSensorV3.RawColor(
        0.497, 0.365, 0.143, 0)  # accurate from ~12.2 cm
    Yellow = ColorSensorV3.RawColor(
        0.317, 0.557, 0.124, 0)  # default color

class ControlPanel:
    
    red = tunable(0, writeDefault=False)
    green = tunable(0, writeDefault=False)
    blue = tunable(0, writeDefault=False)
    currentColorString = tunable("No Color", writeDefault=True)
    turnToColorString = tunable("No message from field", writeDefault=True)
    fmsColorString = tunable("No message from field", writeDefault=True)

    cp_motor: CANSparkMax

    speed = will_reset_to(0)

    def createObjects(self):
        self.getColor = True

        self.colors = [val for val in Colors.__members__.values()]
        self.fmsColor = ColorSensorV3.RawColor(0, 0, 0, 0)

        self.colorSensor = ColorSensorV3(wpilib.I2C.Port.kOnboard)
        self.colorMatcher = ColorMatch()

        for color in self.colors:
            self.colorMatcher.addColorMatch(color)


    def spin(self, spin):
        if speed > 1 or speed < -1:
            raise Exception('you have acheived speeds not possible for a mere mortal')
    
    self.speed = speed

    def execute(self):

        if len(self.ds.getGameSpecificMessage()) > 0 and self.getColor is True:
            self.fmsColorString = self.ds.getGameSpecificMessage()
            if self.color == 'R':
                self.fmsColor = Colors.Red
            elif self.color == 'G':
                self.fmsColor = Colors.Green
            elif self.color == 'B':
                self.fmsColor = Colors.Blue
            elif self.color == 'Y':
                self.fmsColor = Colors.Yellow
            self.getColor = False

            self.fmsColorString = self.colorToString(self.fmsColor)
            colorInt = self.colors.index(self.fmsColor)
            self.turnToColorString = self.colorToString(
                self.colors[(colorInt + 2) % 4])


        try:
            detectedColor = self.colorSensor.getColor()
        except IOError:
            pass
        else:
            self.red = detectedColor.red
            self.blue = detectedColor.blue
            self.green = detectedColor.green

            self.currentColorString = self.colorToString(detectedColor)
            
        self.cp_motor.set(self.speed)

    def colorToString(self, color: ColorSensorV3.RawColor):
        match = self.colorMatcher.matchClosestColor(color)
        return Colors(match.color).name