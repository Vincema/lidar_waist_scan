import utility
import math
import constants
import reader
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


class lidarInfos:
    def __init__(self,distCenter,angle,height):
        self.distCenter = distCenter
        self.angle = angle
        self.height = height
        self.position = utility.point(self.distCenter*math.cos(self.angle*math.pi/180),
                                      self.distCenter*math.sin(self.angle*math.pi/180),
                                      self.height)
        self.pointDatas = []
    
    def save_datas(self,datas,patientHeight):
        self.pointDatas = []
        localToSystemFrameAngle = (self.angle + 180 - 90)%360 # Get the opposite of te angle of the LIDAR

        # Compute the 3D location of each point from the vertical and horizontal angles and the distance 
        # datas = [horizontal_angle, vertical_angle, distance]
        for i in range(len(datas)):
            alpha = datas[i][0]
            theta = datas[i][1]
            dist = datas[i][2]

            c1 = math.cos(localToSystemFrameAngle*math.pi/180)
            c2 = math.cos(theta*math.pi/180)
            c3 = math.cos(alpha*math.pi/180)
            s1 = math.sin(localToSystemFrameAngle*math.pi/180)
            s2 = math.sin(theta*math.pi/180)
            s3 = math.sin(alpha*math.pi/180)

            x = (-c1*s3)-(s1*c2*c3)
            y = (c1*c2*c3)-(s3*s1)
            z = c3*s2

            x *= dist
            y *= dist
            z *= dist

            x += self.distCenter * math.cos(self.angle*math.pi/180)
            y += self.distCenter * math.sin(self.angle*math.pi/180)
            z += self.position.z

            x += constants.lidarsDistFromRotAxis * math.cos(self.angle*math.pi/180) * (1 - math.cos(theta*math.pi/180))
            y += constants.lidarsDistFromRotAxis * math.sin(self.angle*math.pi/180) * (1 - math.cos(theta*math.pi/180))
            z += constants.lidarsDistFromRotAxis * -math.sin(theta*math.pi/180)

            self.pointDatas.append(utility.point(x,y,z))
           
        # Save patient's height
        utility.patientHeight = patientHeight

        
class setOfLidars:
    def __init__(self):
        self.lidarsSet = []
        for i in range(3):
            self.lidarsSet.append(lidarInfos(constants.lidarsDist,constants.lidarsAngle[i],constants.lidarsHeight))

    def read_datas_files(self):
        hgt = reader.read_scan_infos()
        for i in range(3):
            # data,hgt = reader.read_data_single_lidar(i)
            data = reader.read_data_lidar(0)
            self.lidarsSet[i].save_datas(data,hgt)

    def plot_origin_and_lidars_2D(self):
        plt.figure(utility.figMerge)
        plt.plot([0],[0],'cx',ms=5,mew=2)
        lidarPosPointsX = []
        lidarPosPointsY = []
        for i in range(3):
            lidarPosPointsX.append(self.lidarsSet[i].position.x)
            lidarPosPointsY.append(self.lidarsSet[i].position.y)
        plt.plot(lidarPosPointsX,lidarPosPointsY,'rx',label='Lidars',ms=10,mew=2)
    
    def plot_raw_datas(self):
        fig = plt.figure(utility.figRaw)
        ax = fig.add_subplot(111, projection='3d')
        
        ax.set_xlabel('x position (mm)')
        ax.set_ylabel('y position (mm)')
        ax.set_zlabel('z position (mm)')
        
        lidarPosPointsX = []
        lidarPosPointsY = []
        lidarPosPointsZ = []
        for i in range(3):
            lidarPosPointsX.append(self.lidarsSet[i].position.x)
            lidarPosPointsY.append(self.lidarsSet[i].position.y)
            lidarPosPointsZ.append(self.lidarsSet[i].position.z)
        ax.scatter(lidarPosPointsX,lidarPosPointsY,lidarPosPointsZ,c='r',marker='X',label='Lidars')
            
        pointDatasL1X = []
        pointDatasL1Y = []
        pointDatasL1Z = []
        pointDatasL2X = []
        pointDatasL2Y = []
        pointDatasL2Z = []
        pointDatasL3X = []
        pointDatasL3Y = []
        pointDatasL3Z = []
        for i in range(len(self.lidarsSet[0].pointDatas)):
            pointDatasL1X.append(self.lidarsSet[0].pointDatas[i].x)
            pointDatasL1Y.append(self.lidarsSet[0].pointDatas[i].y)
            pointDatasL1Z.append(self.lidarsSet[0].pointDatas[i].z)
        for i in range(len(self.lidarsSet[1].pointDatas)):
            pointDatasL2X.append(self.lidarsSet[1].pointDatas[i].x)
            pointDatasL2Y.append(self.lidarsSet[1].pointDatas[i].y)
            pointDatasL2Z.append(self.lidarsSet[1].pointDatas[i].z)
        for i in range(len(self.lidarsSet[2].pointDatas)):
            pointDatasL3X.append(self.lidarsSet[2].pointDatas[i].x)
            pointDatasL3Y.append(self.lidarsSet[2].pointDatas[i].y)
            pointDatasL3Z.append(self.lidarsSet[2].pointDatas[i].z)
        ax.scatter(pointDatasL1X,pointDatasL1Y,pointDatasL1Z,c='b',marker='.',label='Raw datas Lidar 1')
        ax.scatter(pointDatasL2X,pointDatasL2Y,pointDatasL2Z,c='g',marker='.',label='Raw datas Lidar 2')
        ax.scatter(pointDatasL3X,pointDatasL3Y,pointDatasL3Z,c='k',marker='.',label='Raw datas Lidar 3')
        
        ax.set_xbound(constants.boundsDatasLidars[0],constants.boundsDatasLidars[1])
        ax.set_ybound(constants.boundsDatasLidars[2],constants.boundsDatasLidars[3])
        ax.set_zbound(0,constants.lidarsHeight*2)

    def plot_merged_datas(self):
        global mergedPointsXY

        plt.figure(utility.figMerge)
        
        plt.title('Body shape')
        plt.xlabel('x position (mm)')
        plt.ylabel('y position (mm)')
        
        self.plot_origin_and_lidars_2D()
        mergedPointsX = []
        mergedPointsY = []
        for i in range(len(utility.mergedPointsXY)):
            mergedPointsX.append(utility.mergedPointsXY[i].x)
            mergedPointsY.append(utility.mergedPointsXY[i].y)
        plt.plot(mergedPointsX,mergedPointsY,'k.',label='Raw datas',ms=2.5)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis(constants.boundsDatasLidars)


    def compute_raw_datas(self):
        global mergedPointsXY
        global centeredPointsXY

        # Keep only the datas that are close t the origin
        utility.mergedPointsXY = []
        utility.centeredPointsXY = []
        x = []
        y = []
        for i in range(3):
            for j in range(len(self.lidarsSet[i].pointDatas)):
                if utility.is_useful_data(utility.point(self.lidarsSet[i].pointDatas[j].x,self.lidarsSet[i].pointDatas[j].y,self.lidarsSet[i].pointDatas[j].z)):
                    temp_x = self.lidarsSet[i].pointDatas[j].x
                    temp_y = self.lidarsSet[i].pointDatas[j].y
                    # Remove duplicates
                    if temp_x not in x or temp_y not in y:
                        x.append(self.lidarsSet[i].pointDatas[j].x)
                        y.append(self.lidarsSet[i].pointDatas[j].y)
                    
        for i in range(len(x)):           
            utility.mergedPointsXY.append(utility.point(x[i],y[i]))

        # Plot the merged/centered points
        self.plot_merged_datas()
        
        
                                      
