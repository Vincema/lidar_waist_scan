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
        self.pointData = []
    
    def save_datas(self,data,patientHeight):
        self.pointData = []
        localToSystemFrameAngle = (self.angle + 180 - 90)%360 # Get the opposite of te angle of the LIDAR

        # Compute the 3D location of each point from the vertical and horizontal angles and the distance 
        # datas = [horizontal_angle, vertical_angle, distance]
        for i in range(len(data)):
            alpha = data[i][0]
            theta = data[i][1]
            dist = data[i][2]

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
            z += constants.lidarsDistFromRotAxis * math.sin(theta*math.pi/180)

            self.pointData.append(utility.point(x,y,z))
           
        # Save patient's height
        utility.patientHeight = patientHeight

        
class setOfLidars:
    def __init__(self):
        self.lidarsSet = []
        for i in range(3):
            self.lidarsSet.append(lidarInfos(constants.lidarsDist,constants.lidarsAngle[i],constants.lidarsHeight))

    def read_datas_files(self):
        hgt = reader.read_scan_infos()
        for i in range(constants.nb_of_lidars):
            data = reader.read_data_lidar(i)
            #data = reader.read_data_lidar(0)
            self.lidarsSet[i].save_datas(data,hgt)

    def plot_origin_and_lidars_2D(self):
        plt.figure(utility.figMerge)
        plt.plot([0],[0],'cx',ms=5,mew=2)
        plt.plot(self.lidarsSet[0].position.x,self.lidarsSet[0].position.y,'bx',label='Lidars',ms=10,mew=2)
        plt.plot(self.lidarsSet[1].position.x,self.lidarsSet[1].position.y,'gx',label='Lidars',ms=10,mew=2)
        plt.plot(self.lidarsSet[2].position.x,self.lidarsSet[2].position.y,'kx',label='Lidars',ms=10,mew=2)
    
    def plot_raw_datas(self):
        fig = plt.figure(utility.figRaw)
        ax = fig.add_subplot(111, projection='3d')
        
        ax.set_xlabel('x position (mm)')
        ax.set_ylabel('y position (mm)')
        ax.set_zlabel('z position (mm)')
        
        ax.scatter(self.lidarsSet[0].position.x,self.lidarsSet[0].position.y,self.lidarsSet[0].position.z,c='b',marker='X',s=30,label='Lidars')
        ax.scatter(self.lidarsSet[1].position.x,self.lidarsSet[1].position.y,self.lidarsSet[1].position.z,c='g',marker='X',s=30,label='Lidars')
        ax.scatter(self.lidarsSet[2].position.x,self.lidarsSet[2].position.y,self.lidarsSet[2].position.z,c='k',marker='X',s=30,label='Lidars')

        pointDataL1X = []
        pointDataL1Y = []
        pointDataL1Z = []
        pointDataL2X = []
        pointDataL2Y = []
        pointDataL2Z = []
        pointDataL3X = []
        pointDataL3Y = []
        pointDataL3Z = []
        for i in range(len(self.lidarsSet[0].pointData)):
            pointDataL1X.append(self.lidarsSet[0].pointData[i].x)
            pointDataL1Y.append(self.lidarsSet[0].pointData[i].y)
            pointDataL1Z.append(self.lidarsSet[0].pointData[i].z)
        for i in range(len(self.lidarsSet[1].pointData)):
            pointDataL2X.append(self.lidarsSet[1].pointData[i].x)
            pointDataL2Y.append(self.lidarsSet[1].pointData[i].y)
            pointDataL2Z.append(self.lidarsSet[1].pointData[i].z)
        for i in range(len(self.lidarsSet[2].pointData)):
            pointDataL3X.append(self.lidarsSet[2].pointData[i].x)
            pointDataL3Y.append(self.lidarsSet[2].pointData[i].y)
            pointDataL3Z.append(self.lidarsSet[2].pointData[i].z)
        ax.scatter(pointDataL1X,pointDataL1Y,pointDataL1Z,c='b',marker='.',label='Raw datas Lidar 1')
        ax.scatter(pointDataL2X,pointDataL2Y,pointDataL2Z,c='g',marker='.',label='Raw datas Lidar 2')
        ax.scatter(pointDataL3X,pointDataL3Y,pointDataL3Z,c='k',marker='.',label='Raw datas Lidar 3')
        
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
            for j in range(len(self.lidarsSet[i].pointData)):
                if utility.is_useful_data(utility.point(self.lidarsSet[i].pointData[j].x,self.lidarsSet[i].pointData[j].y,self.lidarsSet[i].pointData[j].z)):
                    x.append(self.lidarsSet[i].pointData[j].x)
                    y.append(self.lidarsSet[i].pointData[j].y)
                    # Remove duplicates
                    #if temp_x not in x or temp_y not in y:
                    #    x.append(self.lidarsSet[i].pointData[j].x)
                    #    y.append(self.lidarsSet[i].pointData[j].y)
                    
        for i in range(len(x)):           
            utility.mergedPointsXY.append(utility.point(x[i],y[i]))

        # Plot the merged/centered points
        self.plot_merged_datas()
        
        
                                      
