import math
import pickle
import sys
from dataclasses import dataclass
from os import path
from typing import Dict, List, Tuple
import logging
from enum import Enum

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
TRACK_WIDTH = 0.51  # Units: meters
MAX_GENERATION_VELOCITY = 2.6  # Units: m/s
MAX_GENERATION_VOLTAGE = 5  # Units: volts

TRAJECTORY_DIRECTORY = 'trajectories'
PICKLE_FILE_NAME = 'trajectories.pickle'
PICKLE_FILE = path.join(
    path.dirname(sys.modules['__main__'].__file__),
    TRAJECTORY_DIRECTORY,
    PICKLE_FILE_NAME
)

# Unimportant due to DifferentialDriveVoltageConstraint
MAX_GENERATION_ACCELERATION = 2  # Units: m/s^2.

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
    field_relative: bool = True
    reverse: bool = False


# The POWER PORT's Pose relative to the top left coordinate of the field as provided by Path weaver.
POWER_PORT = Pose2d(0.2, -2.437, Rotation2d())


# Starting positions are relative to the field's top left coordinate facing down the field
class StartingPosition(Enum):
    # LEFT is one foot off the LEFT wall relative to the Power port (and facing the power port).
    LEFT = Pose2d(3.2, -0.6468, Rotation2d.fromDegrees(180)).relativeTo(POWER_PORT)
    CENTER = Pose2d(3.2, 0, Rotation2d.fromDegrees(180)).relativeTo(POWER_PORT)
    RIGHT = Pose2d(3.2, -3.25, Rotation2d.fromDegrees(180)).relativeTo(POWER_PORT)


# ALL trajectory points should be relative to the field's top left coordinate down the field
# Trajectories that aren't field relative are generated with respect to ALL Starting Positions
# E.x. "charge-LEFT", "charge-CENTER", "charge-RIGHT"
# We do this to allow for trajectory chaining, which requires field-relativity
TRAJECTORIES = {
    "charge": TrajectoryData(
        Pose2d(), [Translation2d(1, 0)], Pose2d(2, 0, Rotation2d()),
        field_relative=False
    ),
    "curve": TrajectoryData(
        Pose2d(), [Translation2d(1, 0.3)], Pose2d(2, 0, Rotation2d()),
        field_relative=False
    ),
    "trench": TrajectoryData(
        Pose2d(3.2, -0.696, Rotation2d.fromDegrees(180)), [Translation2d(6.083, -0.696)],
        Pose2d(7.975, -0.696, Rotation2d.fromDegrees(180)), reverse=True
    ),
    "trench-return": TrajectoryData(
        Pose2d(7.975, -0.696, Rotation2d.fromDegrees(180)), [], Pose2d(6.083, -0.696, Rotation2d.fromDegrees(180))
    ),
    "trench-forward": TrajectoryData(
        StartingPosition.LEFT.value,
        [
            Translation2d(5.402, -0.722),
            Translation2d(6.184, -0.722),
            Translation2d(7.067, -0.722),
            Translation2d(7.621, -0.772),
            Translation2d(7.47, -1.504),
            Translation2d(5.427, -1.176),
            Translation2d(4.216, -1.655)
        ],
        Pose2d(4.822, -2.21, Rotation2d.fromDegrees(185)), reverse=True
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
        logging.getLogger('robot').error(
            'Trajectory file not found. Did you forget to generate them?')
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
        new_trajectories = {}

        if not traj_data.field_relative:
            start_pos: StartingPosition
            for start_pos in StartingPosition:
                # The subtracted Pose2d here is the ORIGIN of the field (The power port).
                # All trajectories are now relative to the POWER PORT.
                transformed_trajectory = generate_trajectory(traj_data).transformBy(start_pos.value - Pose2d())
                new_trajectories[f'{key}-{start_pos.name}'] = transformed_trajectory
        else:
            # Transforms Pathweaver trajectories which are relative to the field's top left coordinate
            # to being relative to the POWER PORT
            new_trajectories[key] = generate_trajectory(traj_data).relativeTo(POWER_PORT)

        # All trajectories should be POWER PORT relative at this point
        generated_trajectories.update({k: TrajectoryUtil.serializeTrajectory(v) for k, v in new_trajectories.items()})

    print('Done')
    print('Writing Trajectories...', end='')

    with open(PICKLE_FILE, 'wb') as f:
        pickle.dump(generated_trajectories, f)

    print('Done')
