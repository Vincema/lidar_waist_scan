import math
import constants
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial
from scipy import cluster


# Misc data arrays
mergedPointsXY = []
clusteredPointsXY = []
centeredPointsXY = []
distancePoints = []
loessPointsIter1 = []
loessPointsIter2 = []
loessResultXY = []
patientHeight = 0

# Misc variables
circum = 0

# Figures
figRaw = 1
figMerge = 2


class point:
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z


class distAngleData:
    def __init__(self,dist=0,alpha=0):
        self.dist = dist
        self.alpha = alpha


def is_useful_data(pointXYZ):
    # Is in the circle of 75% of the radius of the outter circle formed by the lidars 
    if math.sqrt(pointXYZ.x**2 + pointXYZ.y**2) <= (constants.lidarsDist-constants.deadZone):
        # Is at the good height
        if pointXYZ.z >= patientHeight-constants.margin_bot and pointXYZ.z <= patientHeight+constants.margin_top:
            return True
    return False


def compute_circumference():
    global loessResultXY
    global circum

    circum = 0
    loessResultXY
    for i in range(len(loessResultXY)-1):
        distX = loessResultXY[i+1].x - loessResultXY[i].x
        distY = loessResultXY[i+1].y - loessResultXY[i].y
        circum += math.sqrt(distX**2 + distY**2)
    print('\nCircumference: ',format(circum, '.2f'),'mm')

