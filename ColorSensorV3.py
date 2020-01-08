
# TODO: Import from Hal
import struct
from wpilib import DriverStation
from wpilib import I2C

#import edu.wpi.first.wpilibj.util.Color
class Color:
    pass

from enum import IntEnum


class ColorSensorV3:
    kAddress = 82

    kPartID = -62

    def __init__(self, port: I2C.Port):
        self.m_i2c = I2C(port, 82)
        """
        self.m_simDevice = SimDevice.create(
            "REV Color Sensor V3", port.value, 82)
        if self.m_simDevice is not None:
            self.m_simR=self.m_simDevice.createDouble("Red", False, 0.0)
            self.m_simG=self.m_simDevice.createDouble("Green", False, 0.0)
            self.m_simB=self.m_simDevice.createDouble("Blue", False, 0.0)
            self.m_simIR=self.m_simDevice.createDouble("IR", False, 0.0)
            self.m_simProx=self.m_simDevice.createDouble("Proximity", False, 0.0)
            return
        """
        if (not self.checkDeviceID()):
            return
        self.initializeDevice()
        self.hasReset()
    
    class Register(IntEnum):
        kMainCtrl = 0x00
        kProximitySensorLED = 0x01
        kProximitySensorPulses = 0x02
        kProximitySensorRate = 0x03
        kLightSensorMeasurementRate = 0x04
        kLightSensorGain = 0x05
        kPartID = 0x06
        kMainStatus = 0x07
        kProximityData = 0x08
        kDataInfrared = 0x0A
        kDataGreen = 0x0D
        kDataBlue = 0x10
        kDataRed = 0x13
        


    class MainControl(IntEnum):
        kRGBMode = 0x04  #If bit is set to 1, color channels are activated
        kLightSensorEnable = 0x02  #Enable light sensor
        kProximitySensorEnable = 0x01  #Proximity sensor active
        OFF = 0x00   # Nothing on


    class GainFactor(IntEnum):
        kGain1x = 0x00
        kGain3x = 0x01
        kGain6x = 0x02
        kGain9x = 0x03
        kGain18x = 0x04


    class LEDCurrent(IntEnum):
        kPulse2mA = 0x00
        kPulse5mA = 0x01
        kPulse10mA = 0x02
        kPulse25mA = 0x03
        kPulse50mA = 0x04
        kPulse75mA = 0x05
        kPulse100mA = 0x06 #default value
        kPulse125mA = 0x07


    class LEDPulseFrequency(IntEnum):
        kFreq60kHz = 0x18 #default value
        kFreq70kHz = 0x40
        kFreq80kHz = 0x28
        kFreq90kHz = 0x30
        kFreq100kHz = 0x38


    class ProximitySensorResolution(IntEnum):
        kProxRes8bit = 0x00
        kProxRes9bit = 0x01
        kProxRes10bit = 0x02
        kProxRes11bit = 0x03


    class ProximitySensorMeasurementRate(IntEnum):
        kProxRate6ms = 0x01
        kProxRate12ms = 0x02
        kProxRate25ms = 0x03
        kProxRate50ms = 0x04
        kProxRate100ms = 0x05 #default value
        kProxRate200ms = 0x06
        kProxRate400ms = 0x07


    class ColorSensorResolution(IntEnum):
        kColorSensorRes20bit = 0x00
        kColorSensorRes19bit = 0x08
        kColorSensorRes18bit = 0x10
        kColorSensorRes17bit = 0x18
        kColorSensorRes16bit = 0x20
        kColorSensorRes13bit = 0x28


    class ColorSensorMeasurementRate(IntEnum):
        kColorRate25ms = 0
        kColorRate50ms = 1
        kColorRate100ms = 2
        kColorRate200ms = 3
        kColorRate500ms = 4
        kColorRate1000ms = 5
        kColorRate2000ms = 7


    def configureProximitySensorLED(self, freq: LEDPulseFrequency, curr: LEDCurrent, pulses: int):
        self.write8(self.Register.kProximitySensorLED, freq.bVal | curr.bVal)
        self.write8(self.Register.kProximitySensorPulses, pulses)

    def configureProximitySensor(self, res: ProximitySensorResolution, rate: ProximitySensorMeasurementRate):
        self.write8(self.Register.kProximitySensorRate, res.bVal | rate.bVal)

    def configureColorSensor(self, res: ColorSensorResolution, rate: ColorSensorMeasurementRate, gain: GainFactor):
        self.write8(self.Register.kLightSensorMeasurementRate, res.bVal | rate.bVal)
        self.write8(self.Register.kLightSensorGain, gain.bVal)

    def getColor(self):
        r = self.getRed()
        g = self.getGreen()
        b = self.getBlue()
        mag = r + g + b
        return ColorShim(r / mag, g / mag, b / mag)
    
    def getProximity(self):
        """
        if self.m_simDevice is not None:
            return self.m_simProx.get()
        """
        return self.read11BitRegister(self.Register.kProximityData)

    def getRawColor(self):
        return RawColor(self.getRed(), self.getGreen(), self.getBlue(), self.getIR())


    def getRed(self):
        """
        if self.m_simDevice is not None:
            return self.m_simR.get()
        """
        return self.read20BitRegister(self.Register.kDataRed)


    def getGreen(self):
        """
        if self.m_simDevice is not None:
            return self.m_simG.get()
        """
        return self.read20BitRegister(self.Register.kDataGreen)


    def getBlue(self):
        """
        if self.m_simDevice is not None:
            return self.m_simB.get()
        """
        return self.read20BitRegister(self.Register.kDataBlue)


    def getIR(self):
        """
        if self.m_simDevice is not None:
            return self.m_simIR.get()
        """
        return self.read20BitRegister(self.Register.kDataInfrared)

    def hasReset(self):
        #raw = ByteBuffer.allocate(1)
        raw = self.m_i2c.read(self.Register.kMainStatus, 1)
        #raw.order(ByteOrder.LITTLE_ENDIAN)
        raw = struct.pack('<', raw)
        return (raw.get() & 0x20) != 0
    
    def checkDeviceID(self):
        #raw = ByteBuffer.allocate(1)

        """ java code: read doesn't return bool, so I used exception handling instead
        if self.m_i2c.read(self.Register.kPartID, 1):
            DriverStation.reportError(
                "Could not find REV color sensor", False)
            return False
        """
        try:
            raw = self.m_i2c.read(self.Register.kPartID, 1)
        except:
            DriverStation.reportError(
                "Could not find REV color sensor", False)
            return False

        if self.kPartID != raw.get():
            DriverStation.reportError(
                "Unknown device found with same I2C addres as REV color sensor", False)
            return False
        
        return True

    def initializeDevice(self):
        self.write8(self.Register.kMainCtrl,
               self.MainControl.kRGBMode |
               self.MainControl.kLightSensorEnable |
               self.MainControl.kProximitySensorEnable)

        self.write8(self.Register.kProximitySensorRate,
               self.ProximitySensorResolution.kProxRes11bit |
               self.ProximitySensorMeasurementRate.kProxRate100ms)

        self.write8(self.Register.kProximitySensorPulses, 32)
    
    def read11BitRegister(self, reg: Register):
        #raw = ByteBuffer.allocate(2)

        raw = self.m_i2c.read(reg.bVal, 2)

        #raw.order(ByteOrder.LITTLE_ENDIAN)
        raw = struct.pack('<', raw)
        return raw.getShort() & 0x7FF
    
    def read20BitRegister(self, reg: Register):
        #raw = ByteBuffer.allocate(4)

        raw = self.m_i2c.read(reg.bVal, 3)

        #raw.order(ByteOrder.LITTLE_ENDIAN)
        raw = struct.pack('<', raw)
        return raw.getInt() & 0x03FFFF
    
    def write8(self, reg: Register, data: int):
        self.m_i2c.write(reg.bVal, data)
    


class ColorShim:
    def __init__(self, r, g, b):
        super(self, r, g, b)


class RawColor:
    
    def __init__(self, r: int, g: int, b: int, _ir: int):
        self.red = r
        self.green = g
        self.blue = b
        self.ir = _ir
    
