from wpilib.interfaces import SpeedController
from wpilib import Encoder
import traceback
from .fake_impl import FakeImpl

try:
    from rev import CANEncoder, CANSparkMax, MotorType, IdleMode, ControlType, CANPIDController
except ImportError:
    class CANSparkMaxLowLevel(SpeedController, FakeImpl):
        def __init__(self, deviceID: int, type: 'MotorType') -> None:
            super().__init__()
            self.deviceID = deviceID
            self.type = type
    CANSparkMax = type('CANSparkMax', (CANSparkMaxLowLevel, FakeImpl), {})
    MotorType = type('MotorType', (), {'kBrushless': 0, 'kBrushed': 1})
    FaultID = type('FaultId', (), {k: i for i, k in enumerate([
        'kBrownout', 'kCANRX', 'kCANTX', 'kDRVFault', 'kEEPROMCRC',
        'kHardLimitFwd', 'kHardLimitRev', 'kHasReset', 'kIWDTReset',
        'kMotorFault', 'kOtherFault', 'kOvercurrent', 'kSensorFault',
        'kSoftLimitFwd', 'kSoftLimitRev', 'kStall'
    ])})
    ControlType = type('ControlType', (), {'kVelocity': 0, 'kVoltage': 1})
    IdleMode = type('IdleMode', (), {'kBrake': 0, 'kCoast': 1})

    class CANEncoder(Encoder):
        EncoderType = type('EncoderType', (), {'kQuadrature': 0, 'kHallSensor': 1})

    class CANPIDController:
        def setReference(self, value: float, ctrl: ControlType):
            pass
