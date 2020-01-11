import math
from ColorSensorV3 import ColorSensorV3

#TODO import util
#import edu.wpi.first.wpilibj.util.Color
#import edu.wpi.first.wpilibj.util.ColorShim

class ColorMatch:
    def CalculateDistance(self, color1: ColorSensorV3.RawColor, color2: ColorSensorV3.RawColor):
        redDiff = color1.red - color2.red
        greenDiff = color1.green - color2.green
        blueDiff = color1.blue - color2.blue

        return math.sqrt((redDiff*redDiff + greenDiff*greenDiff + blueDiff*blueDiff)/2)

    kDefaultConfidence = 0.95
    colorsToMatch = []

    def __init__(self):
        self.confidenceLevel = self.kDefaultConfidence

    def addColorMatch(self, color: ColorSensorV3.RawColor):
        self.colorsToMatch.append(color)

    def setConfidenceThreshold(self, confidence: float):
        if confidence < 0:
            confidence = 0
        elif confidence > 1:
            confidence = 1
        
        self.confidenceLevel = confidence

    def matchColor(self, colorToMatch: ColorSensorV3.RawColor):
        match = self.matchClosestColor(colorToMatch)
        if match.confidence > self.confidenceLevel:
            return match

        return None

    def makeColor(self, r: float, g: float, b: float):
        return ColorSensorV3.RawColor(r, g, b, 0)

    def matchClosestColor(self, color: ColorSensorV3.RawColor):
        magnitude = color.red + color.blue + color.green

        if magnitude > 0.0 and len(self.colorsToMatch) > 0:
            normalized = ColorSensorV3.RawColor(
                color.red / magnitude, color.green / magnitude, color.blue / magnitude, 0)
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
    def __init__(self, color: ColorSensorV3.RawColor, confidence: float):
        self.color = color
        self.confidence = confidence
