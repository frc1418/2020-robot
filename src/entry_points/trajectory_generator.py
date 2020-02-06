import math
import pickle
import sys
from dataclasses import dataclass
from os import path
from typing import Dict, List, Tuple

from wpilib.controller import SimpleMotorFeedforwardMeters
from wpilib.geometry import Pose2d, Rotation2d, Translation2d
from wpilib.kinematics import DifferentialDriveKinematics
from wpilib.trajectory import (
    Trajectory, TrajectoryConfig,
    TrajectoryGenerator, TrajectoryUtil
)
from wpilib.trajectory.constraint import (
    DifferentialDriveKinematicsConstraint,
    DifferentialDriveVoltageConstraint,
    TrajectoryConstraint
)

from . import robotpy_entry_point

KS = 0.161  # Units: volts
KV = 1.96  # volts * seconds / distance
KA = 0.49  # volts * seconds^2 / distance
TRACK_WIDTH = 0.43  # Units: meters
MAX_GENERATION_VELOCITY = 4  # Units: m/s
MAX_GENERATION_VOLTAGE = 10  # Units: volts

TRAJECTORY_DIRECTORY = 'trajectories'
PICKLE_FILE_NAME = 'trajectories.pickle'
PICKLE_FILE = path.join(
    path.dirname(sys.modules['__main__'].__file__),
    TRAJECTORY_DIRECTORY,
    PICKLE_FILE_NAME
)

# Unimportant due to DifferentialDriveVoltageConstraint
MAX_GENERATION_ACCELERATION = MAX_GENERATION_VELOCITY  # Units: m/s^2.

DRIVE_FEEDFORWARD = SimpleMotorFeedforwardMeters(KS, KV, KA)
KINEMATICS = DifferentialDriveKinematics(TRACK_WIDTH)
DEFAULT_CONSTRAINTS: Tuple[TrajectoryConstraint, ...] = (
    DifferentialDriveKinematicsConstraint(KINEMATICS, MAX_GENERATION_VELOCITY),
    DifferentialDriveVoltageConstraint(DRIVE_FEEDFORWARD, KINEMATICS, MAX_GENERATION_VOLTAGE)
)


@dataclass
class TrajectoryData:
    """Hold metadata for a wpilib trajectory"""
    start: Pose2d
    interior_waypoints: List[Translation2d]
    end: Pose2d
    config: TrajectoryConfig = TrajectoryConfig(MAX_GENERATION_VELOCITY, MAX_GENERATION_ACCELERATION)
    constraints: Tuple[TrajectoryConstraint, ...] = DEFAULT_CONSTRAINTS
    kinematics: DifferentialDriveKinematics = KINEMATICS
    reverse = False


TRAJECTORIES = {
    "charge": TrajectoryData(
        Pose2d(), [Translation2d(1, 0)], Pose2d(2, 0, Rotation2d())
    )
}


def load_trajectories() -> Dict[str, Trajectory]:
    """
    Load trajectories that were saved in the predefined trajectory file.
    """
    try:
        with open(PICKLE_FILE, 'rb') as f:
            generated_trajectories = pickle.load(f)
    except FileNotFoundError:
        print('Trajectory file not found. Did you forget to generate them?')
        generated_trajectories = {}
    else:
        generated_trajectories = {k: TrajectoryUtil.deserializeTrajectory(v) for k, v in generated_trajectories.items()}

    return generated_trajectories


def generate_trajectory(trajectory_data: TrajectoryData) -> Trajectory:
    for constraint in trajectory_data.constraints:
        trajectory_data.config.addConstraint(constraint)

    trajectory_data.config.setKinematics(trajectory_data.kinematics)
    trajectory_data.config.setReversed(trajectory_data.reverse)

    return TrajectoryGenerator.generateTrajectory(
        trajectory_data.start, trajectory_data.interior_waypoints,
        trajectory_data.end, trajectory_data.config
    )


@robotpy_entry_point(name='TrajectoryGenerate')
def generate_trajectories(options, robot_class):
    """
    Generate trajectory from waypoints.
    :param options: Options for the entry point being called
    :param robot_class: The class of the robot being run
    """
    print('Generating Trajectories...', end='')

    generated_trajectories: Dict[str, str] = {}

    traj_data: TrajectoryData
    for key, traj_data in TRAJECTORIES.items():
        trajectory = generate_trajectory(traj_data)

        generated_trajectories[key] = TrajectoryUtil.serializeTrajectory(trajectory)

    print('Done')
    print('Writing Trajectories...', end='')

    with open(PICKLE_FILE, 'wb') as f:
        pickle.dump(generated_trajectories, f)

    print('Done')
