from setuptools import find_packages, setup

setup(
    name='robot_1418',
    version='1.0.0',
    packages=['entry_points'],
    package_dir={'': 'src'},
    install_requires=['wpilib'],
    entry_points={
        'robotpy': [
          'generate=entry_points.trajectory_generator:TrajectoryGenerate'
        ]
    }
)