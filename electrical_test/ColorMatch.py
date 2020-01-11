import math
from ColorSensorV3 import ColorSensorV3

#TODO import util
#import edu.wpi.first.wpilibj.util.Color
#import edu.wpi.first.wpilibj.util.ColorShim
class Color:
    pass

class ColorMatch:
    def CalculateDistance(self, color1: Color, color2: Color):
        redDiff = color1.red - color2.red
        greenDiff = color1.green - color2.green
        blueDiff = color1.blue - color2.blue

        return math.sqrt((redDiff*redDiff + greenDiff*greenDiff + blueDiff*blueDiff)/2)

    kDefaultConfidence = 0.95
    colorsToMatch = []

    def __init__(self):
        self.confidenceLevel = self.kDefaultConfidence

    def addColorMatch(self, color: Color):
        self.colorsToMatch.append(color)

    def setConfidenceThreshold(self, confidence: float):
        if confidence < 0:
            confidence = 0
        elif confidence > 1:
            confidence = 1
        
        self.confidenceLevel = confidence

    def matchColor(self, colorToMatch: Color):
        match = self.matchClosestColor(colorToMatch)
        if match.confidence > self.confidenceLevel:
            return match

        return None

    def makeColor(self, r: float, g: float, b: float):
        return Color(r, g, b)

    def matchClosestColor(self, color: Color):
        magnitude = color.red + color.blue + color.green

        if magnitude > 0.0 and len(self.colorsToMatch) > 0:
            normalized = Color(color.red / magnitude, color.green / magnitude, color.blue / magnitude)
            minDistance = 1.0
            idx = 0

            for i in range(len(self.colorsToMatch)):
                targetDistance = self.CalculateDistance(self.colorsToMatch[i], normalized)

                if targetDistance < minDistance:
                    minDistance = targetDistance
                    idx = i

            match = ColorMatchResult(self.colorsToMatch[idx], 1.0 - minDistance)
            return match
        else:
            #return frc:: Color: : kBlack
            return ColorMatchResult(ColorSensorV3.RawColor(0, 0, 0, 0), 0.0)



class ColorMatchResult:
    def __init__(self, color: Color, confidence: float):
        self.color = color
        self.confidence = confidence
