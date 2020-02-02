from wpilib.interfaces import SpeedController

try:
    from ctre import WPI_TalonSRX, WPI_VictorSPX
except ImportError:
    WPI_TalonSRX = type('WPI_TalonSRX', (SpeedController,), {})
    WPI_VictorSPX = type('WPI_VictorSPX', (SpeedController,), {})
