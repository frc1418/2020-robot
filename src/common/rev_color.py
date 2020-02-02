try:
    from rev.color import ColorMatch, ColorSensorV3, Colors, CIEColor

except ImportError:
    
    class CIEColor():
        pass

    class ColorMatch():
        def addColorMatch(self, *args, **kwargs):
            pass

    class ColorSensorV3():
        def getColor(self, *args, **kwargs):
            return 0

    class Colors():
        pass
