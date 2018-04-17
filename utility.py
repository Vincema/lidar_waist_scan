import math
import constants
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

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
smoothFactor = [None, None]

# Figures
figRaw = 1
figMerge = 2
figDistances = 3


class point:
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z


class distAngleData:
    def __init__(self,dist=0,alpha=0):
        self.dist = dist
        self.alpha = alpha


# Center datas and remove outliers
def center_datas():
    global mergedPointsXY

    global clusteredPointsXY
    clusteredPointsXY = clustering(mergedPointsXY)

    origin = fit_circle(clusteredPointsXY)
    
    global centeredPointsXY
    centeredPointsXY = []
    
    for i in range(len(clusteredPointsXY)):
        centeredPointsXY.append(point(mergedPointsXY[i].x-origin.x,mergedPointsXY[i].y-origin.y))

def get_neighbors(datas,tres,pointIndex,distMatrix):
    neighbors = []
    for i in range(len(datas)):
        if pointIndex != i:
            if distMatrix[pointIndex][i] < tres:
                neighbors.append(i)
    return neighbors


# Clustering points to remove outliers (DBSCAN method)
def clustering(datas):
    ptMin = math.floor(constants.nonNoiseMeas*len(datas))
    a = constants.aStartClust 
    b = constants.bStartClust
    nbInliers = 0
    
    distMatrix = np.zeros((len(datas),len(datas)))
    for i in range(len(datas)):
        for j in range(i,len(datas)):
            dist = math.sqrt((datas[i].x-datas[j].x)**2 + (datas[i].y-datas[j].y)**2)
            distMatrix[i][j] = dist
            distMatrix[j][i] = dist
    
    while b-a > constants.deltaMinClust:
        eps = a+((b-a)/2)
        
        # -1: Undefined
        # 0: Noise
        # >0: Cluster NB
        pointLabel = []
        clust = 0
        
        for i in range(len(datas)):
            pointLabel.append(-1)
        
        for i in range(len(datas)):
            if pointLabel[i] == -1:
                neighbors = get_neighbors(datas,eps,i,distMatrix)
                if len(neighbors) < ptMin:
                    pointLabel[i] = 0
                else:
                    clust = clust+1
                    pointLabel[i] = clust
                    seedSet = []
                    for j in range(len(neighbors)):
                        seedSet.append(neighbors[j])
                    for j in seedSet:
                        if pointLabel[j] == 0:
                            pointLabel[j] = clust
                        if pointLabel[j] == -1:
                            pointLabel[j] = clust
                            neighbors = get_neighbors(datas,eps,i,distMatrix)
                            if len(neighbors) >= ptMin:
                                for k in range(len(neighbors)):
                                    seedSet.append(neighbors[k])
                            
        nbInliers=0
        for i in range(len(datas)):
            if pointLabel[i] != 0:
                nbInliers += 1
        if nbInliers < ptMin:
            a = eps
        else:
            b = eps
            tempPointLabel = pointLabel
            
    newDatas = []
    for i in range(len(datas)):
        if tempPointLabel[i] != 0:
            newDatas.append(datas[i])
    
    return newDatas
            

# Circle fitting
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

    
def is_useful_data(pointXYZ):
    # Is in the circle of 75% of the radius of the outter circle formed by the lidars 
    if math.sqrt(pointXYZ.x**2 + pointXYZ.y**2) <= (0.75 * constants.lidarsDist):
        # Is at the good height
        if pointXYZ.z >= patientHeight-constants.margin_bot and pointXYZ.z <= patientHeight+constants.margin_top:
            return True
    
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
        index = 0
        for f in fSeek:
            print("        f = " , f)
            #try:
            [tmpLoessResults, tmpRobFactor] = loess_regression(tempDatas,f,False,False,[])
            #except:
            #    pass
            fValues.append(f)
            tmpQuadError = 0
            for i in range(len(distancePoints)):
                tmpQuadError += (distancePoints[i].dist - tmpLoessResults[i].dist)**2
            print("            Quadratic error = ",tmpQuadError)
            if index > 0 and tmpQuadError > quadError[index-1] and False:
                print("        Mimimum found")
                break
            else:
                quadError.append(tmpQuadError)
                robFactor.append(tmpRobFactor)
                index += 1

        minVal = np.inf
        for i in range(len(quadError)):
            if quadError[i] < minVal:
                minVal = quadError[i]
                f0 = fValues[i]
                robFactorF0 = robFactor[i]

        tmpLoessResults = []
        fValues = []
        quadError = []
        print("    Optimisation - Step 2")
        index = 0
        for f in fSeek:
            print("        f = " , f)
            #try:
            tmpLoessResults = loess_regression(tempDatas,f,False,True,robFactorF0)
            #except:
            #    pass
            fValues.append(f)
            tmpQuadError = 0
            for i in range(len(distancePoints)):
                tmpQuadError += robFactorF0[i]*((distancePoints[i].dist - tmpLoessResults[i].dist)**2)
            if index > 0 and tmpQuadError > quadError[index-1] and False:
                print("        Mimimum found")
                break
            else:    
                quadError.append(tmpQuadError)
                index += 1
     
        minVal = np.inf
        for i in range(len(quadError)):
            if quadError[i] < minVal:
                minVal = quadError[i]
                smoothFactor[iter] = fValues[i]
                
    print("Optimized smooth factor values = " , format(smoothFactor[0],'.3f') , " and " , format(smoothFactor[1],'.3f'))


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
    [loessPointsIter1, robFactor] = loess_regression(distancePoints,smoothFactor[0],True,True,[])
    [loessPointsIter2, robFactor] = loess_regression(loessPointsIter1,smoothFactor[1],True,True,[])
    
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

    # Computing the equation (least squares min)
    i = list(range(nbPoints))
    
    WG = list(map(lambda x: W[x] * G[x], i))    # Replace the loop/append because this function is called many time
    WGX = list(map(lambda x: WG[x] * X[x], i))
    WGY = list(map(lambda x: WG[x] * Y[x], i))

    mX = sum(WGX)/sum(WG)
    mY = sum(WGY)/sum(WG)

    num = list(map(lambda x: WG[x] * (X[x]-mX) * (Y[x]-mY), i))
    den = list(map(lambda x: WG[x] * (X[x]-mX)**2, i))
        
    param1 = sum(num)/sum(den)
    param2 = mY - mX*param1
    
    return [param1,param2]
    

# robustFactors: None to compute them else, a vector array
def loess_regression(datas,smoothFact,useYi,useRobust=True,robustFactors=[]):
    # Vars init
    robFactor = []
    loessPoints = []
    
    # Number of datas
    nDatas = len(datas)

    # Compute the number of points included by the specified span factor
    nSub = math.ceil(smoothFact * nDatas)
    
    # Make nSub odd if even and =15 if < 15 (It makes no sense to run loess on so few points)
    if nSub < 15:
        nSub = 15
    nSub = nSub - (1 - nSub%2)
    subParametersDist = np.zeros((nDatas,nSub))
    subParametersAngle = np.zeros((nDatas,nSub))
    subParametersAngleWeight = np.zeros((nDatas,nSub))

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

        # Find estimation of each point without the robustness factor at first or with if specified in parameters
        if useRobust == False or robustFactors == []:
            subDistWeight = np.ones(nSub)
        else:
            # For each element in subset array
            k = indMinSub
            subDistWeight = []
            for i in range(nSub):
                # Save the robustness factors value
                subDistWeight.append(robustFactors[k])
                # Update the index of subArray
                k = (k+1)%nDatas
        

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
        if useYi == False:
            # Null weight on the current point
            subAngleWeight[int(np.floor(nSub/2))] = 0

        # Get the parameters of the linear regressions for the current point 
        parameters = linear_weighted_reg(subAngle,subDist,subAngleWeight,subDistWeight)
        subParametersAngle[curPoint] = subAngle
        subParametersDist[curPoint] = subDist
        subParametersAngleWeight[curPoint] = subAngleWeight

        # Compute the new points
        angle = datas[curPoint].alpha
        dist = datas[curPoint].alpha*parameters[0] + parameters[1]
        loessPoints.append(distAngleData(alpha=angle,dist=dist))
      
    
    if robustFactors == []:
        robustFactors = []
        weightedLoessPoints = []
        # For each data point
        for curPoint in range(nDatas):
            # Compute the min and max indexes surrounding the span window 
            indMinSub = int((curPoint-np.floor(nSub/2))%nDatas)
            indMaxSub = int((curPoint+np.floor(nSub/2))%nDatas)

            # For each element in subset array
            k = indMinSub
            subResiduals = []
            for i in range(nSub):
                # Save the residuals of the subset
                subResiduals.append(abs(datas[k].dist - loessPoints[k].dist))
                # Update the index of subArray
                k = (k+1)%nDatas  
          
            # Remove Yi for median if not taken in account
            if useYi == False:
                subResiduals.pop(int(np.floor(nSub/2))) 
                
            medianResiduals = np.median(subResiduals)
            x = abs(datas[curPoint].dist - loessPoints[curPoint].dist) / (6*medianResiduals)
            if x >= 1:
                y = 0
            else:
                y = (1-x**2)**2 
            
            robustFactors.append(y)
        
        # For each data point
        for curPoint in range(nDatas):
            # Compute the min and max indexes surrounding the span window 
            indMinSub = int((curPoint-np.floor(nSub/2))%nDatas)
            indMaxSub = int((curPoint+np.floor(nSub/2))%nDatas)
            
            # For each element in subset array
            k = indMinSub
            subDistWeight = []
            for i in range(nSub):
                # Save the robust factor
                subDistWeight.append(robustFactors[k])
                # Update the index of subArray
                k = (k+1)%nDatas  
            
            parameters = linear_weighted_reg(subParametersAngle[curPoint],subParametersDist[curPoint],subParametersAngleWeight[curPoint],subDistWeight)
            
            # Compute the new points
            angle = datas[curPoint].alpha
            dist = datas[curPoint].alpha*parameters[0] + parameters[1]
            weightedLoessPoints.append(distAngleData(alpha=angle,dist=dist))
        
        if useRobust == False:
            return [loessPoints,robustFactors]
        else:
            return [weightedLoessPoints,robustFactors]  
    else:
        return loessPoints


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






    

