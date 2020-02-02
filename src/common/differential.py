from wpilib.kinematics import ChassisSpeeds
from wpilib.geometry import Pose2d, Rotation2d, Twist2d


class DifferentialDriveKinematics:
    def __init__(self, trackWidthMeters: float):
        self.trackWidthMeters = trackWidthMeters

    def toChassisSpeeds(self, wheelSpeeds: 'DifferentialDriveWheelSpeeds'):
        return ChassisSpeeds(
            (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2, 0,
            (wheelSpeeds.rightMetersPerSecond - wheelSpeeds.leftMetersPerSecond) / self.trackWidthMeters
        )

    def toWheelSpeeds(self, chassisSpeeds: ChassisSpeeds):
        return DifferentialDriveWheelSpeeds(
            chassisSpeeds.vxMetersPerSecond - self.trackWidthMeters / 2 * chassisSpeeds.omegaRadiansPerSecond,
            chassisSpeeds.vxMetersPerSecond + self.trackWidthMeters / 2 * chassisSpeeds.omegaRadiansPerSecond
        )


class DifferentialDriveWheelSpeeds:
    def __init__(self, leftMetersPerSecond: float, rightMetersPerSecond: float):
        self.leftMetersPerSecond = leftMetersPerSecond
        self.rightMetersPerSecond = rightMetersPerSecond

    def normalize(self, attainableMaxSpeedMetersPerSecond: float):
        realMaxSpeed = max(abs(self.leftMetersPerSecond), abs(self.rightMetersPerSecond))

        if realMaxSpeed > attainableMaxSpeedMetersPerSecond:
            self.leftMetersPerSecond = self.leftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond
            self.rightMetersPerSecond = self.rightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond

    def toString(self):
        return f'DifferentialDriveWheelSpeeds(Left: {self.leftMetersPerSecond} m/s, \
                Right: {self.rightMetersPerSecond} m/s)'


class DifferentialDriveOdometry:
    def __init__(self, gyroAngle: Rotation2d, initialPoseMeters=Pose2d()):
        self.poseMeters = initialPoseMeters
        self.gyroOffset = self.poseMeters.getRotation().minus(gyroAngle)
        self.previousAngle = initialPoseMeters.getRotation()

        self.prevLeftDistance = 0.0
        self.prevRightDistance = 0.0

    def resetPosition(self, poseMeters: Pose2d, gyroAngle: Rotation2d):
        self.poseMeters = poseMeters
        self.previousAngle = poseMeters.getRotation()
        self.gyroOffset = poseMeters.getRotation().minus(gyroAngle)

        self.prevLeftDistance = 0.0
        self.prevRightDistance = 0.0

    def getPoseMeters(self):
        return self.poseMeters

    def update(self, gyroAngle: Rotation2d, leftDistanceMeters: float, rightDistanceMeters: float):
        deltaLeftDistance = leftDistanceMeters - self.prevLeftDistance
        deltaRightDistance = rightDistanceMeters - self.prevRightDistance

        self.prevLeftDistance = leftDistanceMeters
        self.prevRightDistance = rightDistanceMeters

        averageDeltaDistance = (deltaLeftDistance + deltaRightDistance) / 2.0
        angle = gyroAngle.plus(self.gyroOffset)

        newPose = self.poseMeters.exp(Twist2d(averageDeltaDistance, 0.0, angle.minus(self.previousAngle).getRadians()))

        self.previousAngle = angle

        self.poseMeters = Pose2d(newPose.getTranslation(), angle)
        return self.poseMeters
