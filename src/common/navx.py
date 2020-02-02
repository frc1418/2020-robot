try:
    import navx
except ImportError:
    class AHRS:
        def create_spi(self):
            return None

    class navx:
        AHRS = AHRS()

        def getAngle(self):
            return 5

        def reset(self):
            return None
