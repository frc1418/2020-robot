from wpilib.interfaces import SpeedController
from wpilib import Encoder

try:
    from rev import CANEncoder, CANSparkMax, MotorType, ControlType
except ImportError:
    CANEncoder = type('CANEncoder', (Encoder,), {})
    CANSparkMax = type('CANSparkMax', (SpeedController,), {})
    MotorType = type('MotorType', (), {'kBrushless': 0, 'kBrushed': 1})
    FaultID = type('FaultId', (), {k: i for i, k in enumerate([
        'kBrownout', 'kCANRX', 'kCANTX', 'kDRVFault', 'kEEPROMCRC',
        'kHardLimitFwd', 'kHardLimitRev', 'kHasReset', 'kIWDTReset',
        'kMotorFault', 'kOtherFault', 'kOvercurrent', 'kSensorFault',
        'kSoftLimitFwd', 'kSoftLimitRev', 'kStall'
    ])})
    ControlType = type('ControlType', (), {'kVelocity': 0, 'kVoltage': 1})
