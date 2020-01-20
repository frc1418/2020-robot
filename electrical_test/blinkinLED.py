from wpilip import Spark

class BlinkinLED(Spark):
    RAINBOW = -0.89
    HEARTBEAT_RED = -0.35
    GREEN = 0.71

    def __init__(self, channel: int) -> None:
        super().__init__(channel)
  
        # full of balls = heartbeat red (-.35)
        # able to pick up balls = lawn green (.71)
        # climbing = rainbow with glitter (-.89)
        # color scanning = OFF
