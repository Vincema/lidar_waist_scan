import utility
import math
import constants
import reader
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np


class lidarInfos:
    def __init__(self,distCenter,angle):
        self.distCenter = distCenter
        self.angle = angle
        self.position = utility.point(self.distCenter*math.cos(self.angle*math.pi/180),
                                      self.distCenter*math.sin(self.angle*math.pi/180))
        self.pointDatas = []
    
    def save_datas(self,datas):
        self.pointDatas = []
        visionAngle = (self.angle + 180)%360 # Get the opposite of this angle
        for i in range(len(datas)):
            x = datas[i][1] * math.cos(((visionAngle + datas[i][0])%360)*math.pi/180) + self.position.x
            y = datas[i][1] * math.sin(((visionAngle + datas[i][0])%360)*math.pi/180) + self.position.y
            
            # Remove duplicates
            temp = utility.point(x,y)
            once = True
            for p in self.pointDatas:
                if p.x == temp.x and p.y == temp.y:
                    once = False
                
            if once == True:
                self.pointDatas.append(temp)

        
class setOfLidars:
    def __init__(self):
        self.lidarsSet = []
        for i in range(3):
            self.lidarsSet.append(lidarInfos(constants.lidarsDist,constants.lidarsAngle[i]))

    def read_datas_files(self):
        for i in range(3):
            self.lidarsSet[i].save_datas(reader.read_data_single_lidar(i))

    def plot_origin_and_lidar(self):
        plt.plot([0],[0],'cx',ms=5,mew=2)
        lidarPosPointsX = []
        lidarPosPointsY = []
        for i in range(3):
            lidarPosPointsX.append(self.lidarsSet[i].position.x)
            lidarPosPointsY.append(self.lidarsSet[i].position.y)
        plt.plot(lidarPosPointsX,lidarPosPointsY,'rx',label='Lidars',ms=10,mew=2)
    
    def plot_raw_datas(self):
        plt.figure(utility.figRaw)
        
        plt.title('Raw datas')
        plt.xlabel('x position (mm)')
        plt.ylabel('y position (mm)')
        
        self.plot_origin_and_lidar()
            
        pointDatasL1X = []
        pointDatasL1Y = []
        pointDatasL2X = []
        pointDatasL2Y = []
        pointDatasL3X = []
        pointDatasL3Y = []
        for i in range(len(self.lidarsSet[0].pointDatas)):
            pointDatasL1X.append(self.lidarsSet[0].pointDatas[i].x)
            pointDatasL1Y.append(self.lidarsSet[0].pointDatas[i].y)
        for i in range(len(self.lidarsSet[1].pointDatas)):
            pointDatasL2X.append(self.lidarsSet[1].pointDatas[i].x)
            pointDatasL2Y.append(self.lidarsSet[1].pointDatas[i].y)
        for i in range(len(self.lidarsSet[2].pointDatas)):
            pointDatasL3X.append(self.lidarsSet[2].pointDatas[i].x)
            pointDatasL3Y.append(self.lidarsSet[2].pointDatas[i].y)
        plt.plot(pointDatasL1X,pointDatasL1Y,'b.',label='Raw datas Lidar 1',ms=2)
        plt.plot(pointDatasL2X,pointDatasL2Y,'g.',label='Raw datas Lidar 2',ms=2)
        plt.plot(pointDatasL3X,pointDatasL3Y,'k.',label='Raw datas Lidar 3',ms=2)

        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis(constants.boundsDatasLidars)
        plt.legend()

    def plot_merged_datas(self):
        global mergedPointsXY
        global centeredPointsXY

        plt.figure(utility.figMerge)
        
        plt.title('Merged datas')
        plt.xlabel('x position (mm)')
        plt.ylabel('y position (mm)')
        
        self.plot_origin_and_lidar()
        mergedPointsX = []
        mergedPointsY = []
        centeredPointsX = []
        centeredPointsY = []
        for i in range(len(utility.mergedPointsXY)):
            mergedPointsX.append(utility.mergedPointsXY[i].x)
            mergedPointsY.append(utility.mergedPointsXY[i].y)
            centeredPointsX.append(utility.centeredPointsXY[i].x)
            centeredPointsY.append(utility.centeredPointsXY[i].y)
        plt.plot(mergedPointsX,mergedPointsY,'k.',label='Raw datas',ms=2)
        plt.plot(centeredPointsX,centeredPointsY,'b.',label='Centered datas',ms=4)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis(constants.boundsDatasLidars)


    def compute_raw_datas(self):
        global mergedPointsXY
        global centeredPointsXY
        global distancePoints
        utility.distancePoints = []

        # Keep only the datas that are close t the origin
        utility.mergedPointsXY = []
        utility.centeredPointsXY = []
        for i in range(3):
            for j in range(len(self.lidarsSet[i].pointDatas)):
                if utility.is_useful_data(utility.point(self.lidarsSet[i].pointDatas[j].x,self.lidarsSet[i].pointDatas[j].y)):
                    utility.mergedPointsXY.append(utility.point(self.lidarsSet[i].pointDatas[j].x,self.lidarsSet[i].pointDatas[j].y))
        # Center the datas according to a circle fitting
        utility.center_datas()

        # Plot the merged/centered points
        self.plot_merged_datas()

        # Compute the angle and distance of the points from the origin (and sort them)
        tempPoints1 = []
        tempPoints2 = []
        for i in range(len(utility.centeredPointsXY)):
            dist = math.sqrt(utility.centeredPointsXY[i].x**2 + utility.centeredPointsXY[i].y**2)
            angle = (math.atan2(utility.centeredPointsXY[i].y,utility.centeredPointsXY[i].x) * 180 / math.pi) + 180
            tempPoints1.append(utility.distAngleData(dist=dist,alpha=angle))
        tempPoints2 = sorted(tempPoints1,key=lambda utility:utility.alpha)
        for i in range(len(tempPoints2)):
            utility.distancePoints.append(tempPoints2[i])  
        
        
                                      
