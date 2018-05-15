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


def euclidian_dist(a,b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    
# Circle fitting
def fit_circle(points):
    n = len(points)
    x = points[:,0]
    y = points[:,1]
    
    # Least squares method
    A = n*np.sum(x**2) - np.sum(x)**2
    B = n*np.sum(x*y) - np.sum(x)*np.sum(y)
    C = n*np.sum(y**2) - np.sum(y)**2
    D = 0.5 * (n*np.sum(x*y**2) - np.sum(x)*np.sum(y**2) + n*np.sum(x**3) - np.sum(x**2)*np.sum(x))
    E = 0.5 * (n*np.sum(y*x**2) - np.sum(y)*np.sum(x**2) + n*np.sum(y**3) - np.sum(y**2)*np.sum(y))

    am = ((D*C)-(B*E)) / ((A*C) - B**2)
    bm = ((A*E)-(B*D)) / ((A*C) - B**2)
    origin = np.array([am,bm])
    radius = np.sum(np.sqrt((x-am)**2 + (y-bm)**2))/n

    return origin,radius

def is_useful_data(pointXYZ):
    # Is in the circle of 75% of the radius of the outter circle formed by the lidars
    if math.sqrt(pointXYZ.x**2 + pointXYZ.y**2) <= (constants.lidarsDist-constants.deadZone):
        # Is at the good height
        if pointXYZ.z >= patientHeight-constants.margin_bot and pointXYZ.z <= patientHeight+constants.margin_top:
            return True
    return False

