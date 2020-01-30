from networktables.util import ntproperty
import math

class Limelight():
    YAW_ANGLE = ntproperty('/limelight/tx',0)
    PITCH_ANGLE = ntproperty('/limelight/ty',0)
    # change with new robot; UNIT = inches
    CAMERA_HEIGHT = 43
    TARGET_HEIGHT = 98.25
    # UNIT = degrees
    CAMERA_ANGLE = 0

    def findDistance(self):
        if self.YAW_ANGLE == 0 or self.PITCH_ANGLE == 0:
            return 0
        else:
            distance = (self.TARGET_HEIGHT/self.CAMERA_HEIGHT)/math.tan(self.CAMERA_ANGLE + self.TY)
            return distance
