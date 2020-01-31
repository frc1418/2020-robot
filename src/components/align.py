from drive import Drive
from common.limelight import Limelight

class Align():
    def createObjects(self):
        self.limelight = Limelight()
        self.drive = Drive()
    
    def execute(self):
        angle = limelight.getYaw()
        self.drive.align(angle, True)
        