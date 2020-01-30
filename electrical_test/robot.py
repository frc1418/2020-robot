import magicbot
import wpilib
import wpilib.drive
import math
import time
import random
from networktables.util import ntproperty
from magicbot import tunable

from robotpy_ext.misc.periodic_filter import PeriodicFilter, logging



class TestRobot(magicbot.MagicRobot):

    red = tunable(0, writeDefault=False)
    green = tunable(0, writeDefault=False)
    blue = tunable(0, writeDefault=False)
    currentColorString = tunable("", writeDefault=False)
    turnToColorString = tunable("", writeDefault=False)
    fmsColorString = tunable("", writeDefault=False)
    pot_angle = ntproperty('/robot/pot_angle', 0)
    
    def createObjects(self):
        self.pot = wpilib.AnalogInput(0)

    def teleopPeriodic(self):
        self.pot_angle = self.pot.getVoltage()

if __name__ == '__main__':
    wpilib.run(TestRobot)
