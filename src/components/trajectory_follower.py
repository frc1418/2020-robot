from wpilib.controller import RamseteController, SimpleMotorFeedforwardMeters, PIDController
from wpilib.kinematics import DifferentialDriveKinematics, ChassisSpeeds, DifferentialDriveWheelSpeeds
from wpilib.geometry import Pose2d, Rotation2d, Translation2d
from wpilib.trajectory import Trajectory
from magicbot import will_reset_to, tunable
import math
from . import Odometry, Drive


class Follower:
    kinematics: DifferentialDriveKinematics
    drive_feedforward: SimpleMotorFeedforwardMeters
    trajectories: dict
    odometry: Odometry
    drive: Drive
    use_pid = tunable(True)
    trajectory = will_reset_to(None)
    sample_time = will_reset_to(0)

    def setup(self):
        self.trajectory_name = None
        self.left_controller = PIDController(1, 0, 0)
        self.right_controller = PIDController(1, 0, 0)

    def setup_trajectory(self, trajectory):
        self.controller = RamseteController()
        self.left_controller.reset()
        self.right_controller.reset()
        self.controller.setTolerance(Pose2d(0.1, 0.1, Rotation2d(math.radians(5))))
        self.speed = DifferentialDriveWheelSpeeds()
        self.prev_time = 0
        self.initial_state = self.trajectory.sample(0)
        self.prevSpeeds = self.kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(self.initial_state.velocity, 0, 0, Rotation2d()))

    def follow_trajectory(self, trajectory_name, sample_time):
        self.trajectory = self.trajectories[trajectory_name]
        self.sample_time = sample_time

        if self.trajectory_name != trajectory_name:
            self.setup_trajectory(self.trajectory)

        self.trajectory_name = trajectory_name

    def is_finished(self):
        return self.controller.atReference()

    def execute(self):
        if self.trajectory is None:
            return

        if self.use_pid:
            if self.prev_time != 0:
                self.dt = self.sample_time - self.prev_time
            else:
                self.dt = 1

            targetWheelSpeeds = self.kinematics.toWheelSpeeds(self.controller.calculate(self.odometry.get_pose(), self.trajectory.sample(self.sample_time)))

            leftSpeedSetpoint = targetWheelSpeeds.left
            rightSpeedSetpoint = targetWheelSpeeds.right

            leftFeedforward = self.drive_feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - self.prevSpeeds.left) / self.dt)

            rightFeedforward = self.drive_feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - self.prevSpeeds.right) / self.dt)

            leftOutput = leftFeedforward + self.left_controller.calculate(self.odometry.left_rate, leftSpeedSetpoint)

            rightOutput = rightFeedforward + self.right_controller.calculate(self.odometry.right_rate, rightSpeedSetpoint)
        else:
            leftOutput = leftSpeedSetpoint
            rightOutput = rightSpeedSetpoint

        self.drive.voltageDrive(leftOutput, rightOutput)
        print(f'Left: {leftOutput} Right: {rightOutput}')

        self.prev_time = self.sample_time
        self.prevSpeeds = targetWheelSpeeds
