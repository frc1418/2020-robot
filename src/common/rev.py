from wpilib.interfaces import SpeedController
from wpilib import Encoder
from enum import Enum

try:
    from rev import CANEncoder, CANSparkMax, MotorType, FaultID
except ImportError:
    CANEncoder = type('CANEncoder', (Encoder,), {})
    CANSparkMax = type('CANSparkMax', (SpeedController,), {})
    MotorType = type('MotorType', (Enum,), {'kBrushless': 0, 'kBrushed': 1})
    FaultID = type('FaultId', (Enum,), {k: i for i, k in enumerate([
        'kBrownout', 'kCANRX', 'kCANTX', 'kDRVFault', 'kEEPROMCRC',
        'kHardLimitFwd', 'kHardLimitRev', 'kHasReset', 'kIWDTReset',
        'kMotorFault', 'kOtherFault', 'kOvercurrent', 'kSensorFault',
        'kSoftLimitFwd', 'kSoftLimitRev', 'kStall'
    ])})