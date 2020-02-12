from networktables.util import ntproperty
import math


class Limelight():
    yaw_angle = ntproperty('/limelight/tx', 0)
    pitch_angle = ntproperty('/limelight/ty', 0)
    light_mode = ntproperty('/limelight/ledMode', 0)
    valid_target = ntproperty('/limelight/tv', 0)
    camera_mode = ntproperty('/limelight/camMode', 0)
    pipeline_number = ntproperty('/limelight/pipeline', 0, writeDefault=False)
    target_state = ntproperty('/limelight/target_state', 0)

    # change with new robot; UNIT = inches
    CAMERA_HEIGHT = 16.5
    TARGET_HEIGHT = 98.25
    # UNIT = degrees
    CAMERA_ANGLE = 13.5

    def findPlaneDistance(self):
        if not self.targetExists():
            return 0
        else:
            plane_distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / math.tan(math.radians(self.CAMERA_ANGLE + self.pitch_angle))
            return plane_distance

    def findDirectDistance(self):
        if not self.targetExists():
            return 0
        else:
            direct_distance = (self.TARGET_HEIGHT - self.CAMERA_HEIGHT) / math.sin(math.radians(self.CAMERA_ANGLE + self.pitch_angle))
            return direct_distance

    def getYaw(self):
        return self.yaw_angle

    def setLightMode(self, mode: int):
        # 0: LED mode in pipeline
        # 1: force off
        # 2: force blink
        # 3: force on
        self.light_mode = mode

    def setCamMode(self, mode: bool):
        # true: Driver Camera
        # false: Vision Processor
        self.camera_mode = int(mode)

    def targetExists(self):
        return bool(self.valid_target)

    def changePipieline(self, mode: int):
        # 0: close distance pipeline
        # 1: medium distance pipeline
        # 2: far distance pipeline
        self.pipeline_number = mode