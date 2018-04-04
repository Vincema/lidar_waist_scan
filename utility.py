import math
import constants
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

# Misc data arrays
mergedPointsXY = []
centeredPointsXY = []
distancePoints = []
loessPointsIter1 = []
loessPointsIter2 = []
loessResultXY = []

# Misc variables
circum = 0
smoothFactor = [None, None]

# Figures
figRaw = 1
figMerge = 2
figDistances = 3


class point:
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y


class distAngleData:
    def __init__(self,dist=0,alpha=0):
        self.dist = dist
        self.alpha = alpha


def center_datas():
    global mergedPointsXY

    origin = fit_circle(mergedPointsXY)
    
    global centeredPointsXY
    centeredPointsXY = []
    
    for i in range(len(mergedPointsXY)):
        centeredPointsXY.append(point(mergedPointsXY[i].x-origin.x,mergedPointsXY[i].y-origin.y))


def fit_circle(datas):
    n = len(datas)
    x = []
    y = []
    xSquare = []
    ySquare = []
    xTimey = []
    xSquareTimey = []
    xTimeySquare = []
    xCube = []
    yCube = []
    for i in range(n):
        x.append(datas[i].x)
        y.append(datas[i].y)
        xSquare.append(datas[i].x**2)
        ySquare.append(datas[i].y**2)
        xTimey.append(datas[i].x*datas[i].y)
        xSquareTimey.append(datas[i].x**2 * datas[i].y)
        xTimeySquare.append(datas[i].x * datas[i].y**2)
        xCube.append(datas[i].x**3)
        yCube.append(datas[i].y**3)
    
    # Least squares method
    A = n*sum(xSquare) - sum(x)**2
    B = n*sum(xTimey) - sum(x)*sum(y)
    C = n*sum(ySquare) - sum(y)**2
    D = 0.5 * (n*sum(xTimeySquare) - sum(x)*sum(ySquare) + n*sum(xCube) - sum(xSquare)*sum(x))
    E = 0.5 * (n*sum(xSquareTimey) - sum(y)*sum(xSquare) + n*sum(yCube) - sum(ySquare)*sum(y))
    origin = point((D*C - B*E)/(A*C - B**2) , (A*E - B*D)/(A*C - B**2))
    return origin

    
def is_useful_data(pointXY):
    if math.sqrt(pointXY.x**2 + pointXY.y**2) <= (0.75 * min(constants.lidarsDist)):
        return True
    else:
        return False


def plot_distances():
    global distancePoints
    global loessPointsIter1
    global loessPointsIter2

    plt.figure(figDistances)
    plt.title('Distance from origin')
    plt.xlabel('Angle (deg)')
    plt.ylabel('Distance from origin (mm)')
    
    distancePointsX = []
    distancePointsY = []
    loessPointsIter1X = []
    loessPointsIter1Y = []
    loessPointsIter2X = []
    loessPointsIter2Y = []
    for i in range(len(distancePoints)):
        distancePointsX.append(distancePoints[i].alpha)
        distancePointsY.append(distancePoints[i].dist)
        loessPointsIter1X.append(loessPointsIter1[i].alpha)
        loessPointsIter1Y.append(loessPointsIter1[i].dist)
        loessPointsIter2X.append(loessPointsIter2[i].alpha)
        loessPointsIter2Y.append(loessPointsIter2[i].dist)
    plt.plot(distancePointsX,distancePointsY,'b-',label='Raw datas',ms=2)
    plt.plot(loessPointsIter1X,loessPointsIter1Y,'g-',label='LOESS points iter 1',ms=2)
    plt.plot(loessPointsIter2X,loessPointsIter2Y,'r-',label='LOESS points iter 2',ms=2)
    plt.legend()


def plot_results_loess():
    global loessResultXY
    
    plt.figure(figMerge)

    loessResultX = []
    loessResultY = []
    for i in range(len(loessResultXY)):
        loessResultX.append(loessResultXY[i].x)
        loessResultY.append(loessResultXY[i].y)
    plt.plot(loessResultX,loessResultY,'r-',label='Smoothed points after LOESS reg',ms=2)
    plt.legend()


def find_optimal_smooth_factor():
    global distancePoints
    n = len(distancePoints)
    global smoothFactor
    
    fSeek = []
    fSeek = np.linspace(constants.fMin,constants.fMax,constants.stepNb)

    tempDatas = distancePoints
    
    for iter in range(2):
        print("Optimisation - Iter ",iter+1)
        tmpLoessResults = []
        fValues = []
        quadError = []
        robFactor = []
        print("    Optimisation - Step 1")
        index = 1
        for f in fSeek:
            print("    Iter " , index , '/' , constants.stepNb)
            index += 1
            [tmpLoessResults, tmpRobFactor] = loess_regression(tempDatas,f,0,0)
            fValues.append(f)
            tmpQuadError = 0
            for i in range(len(distancePoints)):
                tmpQuadError += (distancePoints[i].dist - tmpLoessResults[i].dist)**2
            quadError.append(tmpQuadError)
            robFactor.append(tmpRobFactor)

        minVal = np.inf
        for i in range(constants.stepNb):
            if quadError[i] < minVal:
                minVal = quadError[i]
                f0 = fValues[i]
                robFactorF0 = robFactor[i]

        tmpLoessResults = []
        fValues = []
        quadError = []
        print("    Optimisation - Step 2")
        index = 1
        for f in fSeek:
            print("    Iter " , index , '/' , constants.stepNb)
            index += 1
            [tmpLoessResults, tmpRobFactor] = loess_regression(tempDatas,f,0,1)
            fValues.append(f)
            tmpQuadError = 0
            for i in range(len(distancePoints)):
                tmpQuadError += robFactorF0[i]*((distancePoints[i].dist - tmpLoessResults[i].dist)**2)
            quadError.append(tmpQuadError)
     
        minVal = np.inf
        for i in range(constants.stepNb):
            if quadError[i] < minVal:
                minVal = quadError[i]
                smoothFactor[iter] = fValues[i]

        [tmpLoessResults, tmpRobFactor] = loess_regression(distancePoints,smoothFactor[iter],0,1)
        tempDatas = tmpLoessResults

    print("Optimized smooth factor values = " , format(smoothFactor[0],'.2f') , " and " , format(smoothFactor[1],'.2f'))


def loess_algorithm():
    global distancePoints
    global loessPointsIter1
    loessPointsIter1 = []
    global loessPointsIter2
    loessPointsIter2 = []
    global loessResultXY
    loessResultXY = []          

    # 2 iterations of the loess algorithm
    loessPointsIter1 = []
    loessPointsIter2 = []
    [loessPointsIter1, robFactor] = loess_regression(distancePoints,smoothFactor[0],1,1)
    [loessPointsIter2, robFactor] = loess_regression(loessPointsIter1,smoothFactor[1],1,1)
    
    plot_distances()

    # Compute and plot results of the loess reg on XY 
    for i in range(len(loessPointsIter2)):
        x = loessPointsIter2[i].dist * math.cos((loessPointsIter2[i].alpha - 180)*math.pi/180) 
        y = loessPointsIter2[i].dist * math.sin((loessPointsIter2[i].alpha - 180)*math.pi/180)
        loessResultXY.append(point(x,y))
    loessResultXY.append(loessResultXY[0]) # To close the shape
    plot_results_loess()


def linear_weighted_reg(X,Y,W,G):
    # Number of points
    nbPoints = len(X)

    # Computing the equation (leat squares min)
    WGX = []
    WGY = []
    WG = []
    for i in range(nbPoints):
        WGX.append(W[i] * G[i] * X[i])
        WGY.append(W[i] * G[i] * Y[i])
        WG.append(W[i] * G[i])

    mX = sum(WGX)/sum(WG)
    mY = sum(WGY)/sum(WG)

    num = []
    den = []
    for i in range(nbPoints):
        num.append(W[i] * G[i] * (X[i]-mX) * (Y[i]-mY))
        den.append(W[i] * G[i] * (X[i]-mX)**2)

    param1 = sum(num)/sum(den)
    param2 = mY - mX*param1
    return [param1,param2]
    

def loess_regression(datas,smoothFact,useYi,useRobust):
    # Vars init
    robFactor = []
    loessPoints = []

    # Number of datas
    nDatas = len(datas)

    # Compute the number of points included by the specified span factor
    nSub = math.ceil(smoothFact * nDatas)
    
    # Make nSub odd if even and =3 if < 3
    if nSub < 3:
        nSub = 3
    nSub = nSub - (1 - nSub%2)

    # For each data point
    for curPoint in range(nDatas):
        # Compute the min and max indexes surrounding the span window 
        indMinSub = int((curPoint-np.floor(nSub/2))%nDatas)
        indMaxSub = int((curPoint+np.floor(nSub/2))%nDatas)

        # For each element in subset array
        k = indMinSub
        subAngle = []
        subDist = []
        for i in range(nSub):
            # Define the new angle to avoid discontinuities between 0 and 360 degrees
            diffAngle = ((datas[k].alpha - datas[curPoint].alpha+180)%360)-180
            subAngle.append(datas[curPoint].alpha + diffAngle)
            # Save the distance value
            subDist.append(datas[k].dist)
            # Update the index of subArray
            k = (k+1)%nDatas

        # Compute robust weighted regression
        subDistWeight = []
        tempSubDist = subDist[:]
        if useYi == 0:
            tempSubDist.pop(int(np.floor(nSub/2)))  # Remove Yi for median and mean equation if not taken in account
        meanDist = np.mean(tempSubDist)
        medianErrToMEan = np.median(abs(tempSubDist-meanDist))
        # For each element in subset array
        for i in range(nSub):
            errToMean = abs(subDist[i]-meanDist)
            subDistWeight.append(abs(errToMean/(6*medianErrToMEan)))
            if subDistWeight[i] >= 1:
                subDistWeight[i] = 0
            else:
                subDistWeight[i] = (1-subDistWeight[i]**2)**2

        # Save the robustness factor for each point
        robFactor.append(subDistWeight[int(np.floor(nSub/2))])

        # Determine if the user wants to compute the robustess factor
        if useRobust == 0:
            subDistWeight = np.ones(nSub)    

        # Compute the maximum relative angle between the current point and the points in the subset
        rangeAngle = []
        for i in range(len(subAngle)):
            rangeAngle.append(abs((datas[curPoint].alpha - subAngle[i] + 180)%360 - 180)) 
        maxRange = max(rangeAngle)

        # For each element in subset array
        k = indMinSub
        subAngleWeight = []
        for i in range(nSub):
            # Distance weight
            angleDiff = abs((datas[curPoint].alpha - subAngle[i] + 180)%360 - 180)
            scaledDist = angleDiff/maxRange
            subAngleWeight.append((1-scaledDist**3)**3)
            # Update the index of subArray
            k = (k+1)%nDatas

        # If the current y should not be taken into account 
        if useYi == 0:
            # Null weight on the current point
            subAngleWeight[int(np.floor(nSub/2))] = 0

        # Get the parameters of the linear regressions for the current point 
        parameters = linear_weighted_reg(subAngle,subDist,subAngleWeight,subDistWeight)

        # Compute the new points
        angle = datas[curPoint].alpha
        dist = datas[curPoint].alpha*parameters[0] + parameters[1]
        loessPoints.append(distAngleData(alpha=angle,dist=dist))
    return [loessPoints,robFactor]


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






    

