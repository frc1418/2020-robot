from networktables.util import ntproperty
import math
from wpilib.geometry import Pose2d, Translation2d, Rotation2d
import logging
from typing import Optional


class Limelight():
    yaw_angle = ntproperty('/limelight/tx', 0)
    pitch_angle = ntproperty('/limelight/ty', 0)
    light_mode = ntproperty('/limelight/ledMode', 0)
    valid_target = ntproperty('/limelight/tv', 0)
    skew = ntproperty('/limelight/ts', 0)
    camera_mode = ntproperty('/limelight/camMode', 0)
    pipeline_number = ntproperty('/limelight/pipeline', 1)
    target_state = ntproperty('/limelight/target_state', 0)
    pose_data = ntproperty('/limelight/camtran', [0] * 6)

    # change with new robot; UNIT = inches
    CAMERA_HEIGHT = 16.5
    TARGET_HEIGHT = 98.25
    # UNIT = degrees
    CAMERA_ANGLE = 13.5

    def __init__(self):
        self.avg_x = 0
        self.avg_y = 0
        self.avg_rot = 0

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

    def TurnLightOn(self, on: bool):
        if on:
            self.setLightMode(3)
        else:
            self.setLightMode(0)

    def setCamMode(self, mode: bool):
        # true: Driver Camera
        # false: Vision Processor
        self.camera_mode = int(mode)

    def targetExists(self):
        return bool(self.valid_target)

    def changePipeline(self, mode: int):
        # 0: close distance pipeline
        # 1: medium distance pipeline
        # 2: far distance pipeline
        self.pipeline_number = mode

    def getPose(self) -> Optional[Pose2d]:
        if not self.targetExists():
            return None

        data = self.pose_data

        # Turn limelight pose information into a wpilib Pose2d
        rot = Rotation2d.fromDegrees((data[4] + 180) % 360)
        x = 15 * math.cos(rot.degrees()) + (-data[2])
        y = 15 * math.cos(90 - rot.degrees()) + data[0]
        # Change unit from inches to meters
        return Pose2d(x / 39.37, y / 39.37, rot)

    def averagePose(self) -> None:
        pose = self.getPose()

        if pose is None:
            return

        # Compute new average with the current average holding a worth of 9 values
        self.avg_x = ((9 * self.avg_x) + pose.translation().X()) / 10
        self.avg_y = ((9 * self.avg_y) + pose.translation().Y()) / 10
        self.avg_rot = ((9 * self.avg_rot) + pose.rotation().degrees()) / 10

    def getAveragedPose(self) -> Pose2d:
        return Pose2d(self.avg_x, self.avg_y, Rotation2d.fromDegrees(self.avg_rot))
