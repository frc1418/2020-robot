from wpilib.controller import RamseteController, SimpleMotorFeedforwardMeters, PIDController
from wpilib.kinematics import DifferentialDriveKinematics, ChassisSpeeds, DifferentialDriveWheelSpeeds
from wpilib.geometry import Pose2d, Rotation2d, Translation2d
from wpilib.trajectory import Trajectory
from magicbot import will_reset_to, tunable
from drive import Drive

from . import Odometry


class Follower:
    kinematics: DifferentialDriveKinematics
    feedforward: SimpleMotorFeedforwardMeters
    trajectories: dict
    odometry: Odometry
    drive: Drive
    use_pid = tunable(True)
    trajectory = will_reset_to(None)
    sample_time = will_reset_to(0)

    def setup(self):
        self.trajectory_name = None
        self.left_controller = PIDController()
        self.right_controller = PIDController()

    def setup_trajectory(self, trajectory):
        self.controller = RamseteController()
        self.left_controller.reset()
        self.right_controller.reset()
        self.controller.setTolerance(Pose2d(0.1, 0.1, Rotation2d(5)))
        self.speed = DifferentialDriveWheelSpeeds()
        self.prev_time = 0
        self.runnning = True
        self.initial_state = self.trajectory.sample(0)
        self.prevSpeeds = self.kinematics.toWheelSpeeds(
            ChassisSpeeds(self.initial_state.velocityMetersPerSecond,
            0,
            self.initial_state.curvatureRadPerMeter
                * self.initial_state.velocityMetersPerSecond))

    def follow_trajectory(self, trajectory_name, sample_time):
        self.trajectory = self.trajectories[trajectory_name]
        self.sample_time = sample_time

        if self.trajectory_name != trajectory_name:
            self.setup_trajectory(self.trajectory)
        
        self.trajectory_name = trajectory_name

    def is_finished(self):
        return not self.runnning

    def execute(self):
        if self.trajectory is None or self.runnning is False:
            return

        if self.use_pid:
            if self.prev_time != 0:
                self.dt = self.sample_time - self.prev_time
            else:
                self.dt = 1

            targetWheelSpeeds = self.kinematics.toWheelSpeeds(self.controller.calculate(self.odometry.get_pose(), self.trajectory.sample(self.sample_time)))

            leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond
            rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond

            leftFeedforward = self.feedforward.calculate(leftSpeedSetpoint, 
                    (leftSpeedSetpoint - self.prevSpeeds.leftMetersPerSecond) / self.dt)

            rightFeedforward = self.feedforward.calculate(rightSpeedSetpoint,
                (rightSpeedSetpoint - self.prevSpeeds.rightMetersPerSecond) / self.dt)

            leftOutput = leftFeedforward
            + self.left_controller.calculate(self.speed.get().leftMetersPerSecond,
            leftSpeedSetpoint)

            rightOutput = rightFeedforward
            + self.right_controller.calculate(self.speed.get().rightMetersPerSecond,
            rightSpeedSetpoint)
        else:
            leftOutput = leftSpeedSetpoint
            rightOutput = rightSpeedSetpoint

        if self.controller.atReference():
            self.running = False

        self.drive.voltageDrive(leftOutput, rightOutput)
        
        self.prev_time = self.sample_time