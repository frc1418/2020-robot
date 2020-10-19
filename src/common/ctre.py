from wpilib.interfaces import SpeedController
from .fake_impl import FakeImpl

try:
    from ctre import WPI_TalonSRX, WPI_VictorSPX
except ImportError:
    class WPI_TalonSRX(SpeedController, FakeImpl):
        def __init__(self, id):
            super().__init__()

    class WPI_VictorSPX(SpeedController, FakeImpl):
        def __init__(self, id):
            super().__init__()

    # WPI_VictorSPX = type('WPI_VictorSPX', (SpeedController,), {})
