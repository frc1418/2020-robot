from components.drive import Drive
from common.limelight import Limelight
from magicbot import will_reset_to


class Align:
    limelight: Limelight
    drive: Drive
    aligning = will_reset_to(False)
    angle = will_reset_to(0)

    def execute(self):
        if self.aligning == True:
            self.drive.align(self.angle, True)

    def align(self, angle):
        self.aligning = True
        self.angle = angle
        