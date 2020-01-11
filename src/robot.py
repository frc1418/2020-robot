import magicbot
import wpilib
import navx


r"""
/ \                / \
\ /       (+)      \ /
           |
           |X
(+) -------|--Y----  (-)
           |
           |
/ \       (-)      / \
\ /                \ /

Counter-Clockwise is Positive
   /-\ ^
   |X| | (+)
   \-/ |
   -->
"""


class Robot(magicbot.MagicRobot):
    def createObjects(self):
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()


if __name__ == '__main__':
    wpilib.run(Robot)
