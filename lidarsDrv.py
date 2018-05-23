from drivers.rplidar import RPLidar
from pyax12.connection import Connection
import numpy as np
import constants
import time
import math

BROADCAST_ID = 254

# For debug
p = np.array([0.17897664, 1.4094397, 5.8093253])

def compute_angle_array_for_scan(height):
    angle_array = []
    meas_nb = []
    
    # If the patient's height is at the lidar level, no need to change the angle 
    if abs(constants.lidarsHeight - height) <= constants.margin_top_meas or abs(constants.lidarsHeight - height) <= constants.margin_bot_meas:
        angle_array.append(0)
        meas_nb.append(constants.nbOfDatasToRetrieve)
        return [angle_array,meas_nb]
    
    hgt_diff_mtop = constants.lidarsHeight - (height + constants.margin_top_meas)
    hgt_diff_mbot = constants.lidarsHeight - (height - constants.margin_bot_meas)
    
    min_meas_dist = constants.deadZone
    max_meas_dist = 2*(constants.lidarsDist-constants.deadZone)
    
    if height > constants.lidarsHeight:
        hgt1 = hgt_diff_mbot
        hgt2 = hgt_diff_mtop
    else:
        hgt1 = hgt_diff_mtop
        hgt2 = hgt_diff_mbot
    
    currentDist = min_meas_dist
    while currentDist < max_meas_dist:
        dist1 = currentDist
        angle = math.atan2(hgt1,dist1)
        dist2 = hgt2/math.tan(angle)
        currentDist += (dist2-dist1)
        angle_array.append(-angle*180/math.pi)
        if dist2 > max_meas_dist:
            nb = math.ceil(constants.nbOfDatasToRetrieve * (max_meas_dist-dist1) / (max_meas_dist-min_meas_dist))
        else:
            nb = math.ceil(constants.nbOfDatasToRetrieve * (dist2-dist1) / (max_meas_dist-min_meas_dist))
        meas_nb.append(nb)

    angle_array.reverse()
    meas_nb.reverse()
    return [angle_array,meas_nb]
    

# Documentation: http://rplidar.readthedocs.io/en/latest/
class driverLidars:
    def __init__(self):
        self.lidars = []
        self.serial_connection = []
        self.areConnected = 0

    def connect(self):
        print('Connecting to lidars and servos...')
        self.lidars = []
        self.serial_connection = []
        try:
            # Connect to the serial port
            self.serial_connection = Connection(port="/dev/ttyAMA0", baudrate=57600, rpi_gpio=True)
        except:
            print('    Cannot open serial connection for servos!')
            return -1
        for lidarNb in range(constants.nb_of_lidars):
            try:
                self.lidars.append(RPLidar(constants.serialPort[lidarNb],baudrate=115200))
                self.lidars[lidarNb].connect()
            except:
                print('    Cannot connect to the lidar',lidarNb+1,'!')
                return -1
            print('    Connected to lidar',lidarNb+1)
            
            try:
                # Try to ping the motor
                if self.serial_connection.ping(constants.servosIDs[lidarNb]) == False:
                    raise
                self.serial_connection.set_speed(BROADCAST_ID,constants.servosSpeed)
                self.servos_goto(constants.servosIDs[lidarNb],0)
            except:
                print('    Cannot connect to the servo',lidarNb+1,'!')
                return -1
            print('    Connected to servo',lidarNb+1)
            time.sleep(0.25) # To avoid a too high current drain
        self.areConnected = 1
        return 0
        
    def disconnect(self):
        print('Disconnecting to lidars and servos...')
        self.serial_connection.close()
        print('    Disconnected to servos serial')
        for lidarNb in range(constants.nb_of_lidars):
            self.lidars[lidarNb].stop()
            self.lidars[lidarNb].disconnect()
            time.sleep(0.25) # To avoid to drain too much current
            print('    Disconnected to lidar',lidarNb+1)
        self.lidars[:] = []
        self.areConnected = 0

    def servos_goto(self,servosID,position):
        cmd = np.polyval(p,position)
        if cmd <= constants.limit_cw_angle and cmd >= constants.limit_ccw_angle:
            sat_cmd = cmd
            true_angle = position
        else:
            if cmd > constants.limit_cw_angle:
                sat_cmd = constants.limit_cw_angle
            else:
                sat_cmd = constants.limit_ccw_angle
            
            true_angle = np.roots(np.append(p[0:-1],p[-1]-sat_cmd))[1]
        self.serial_connection.goto(servosID,sat_cmd,degrees=True)
        #print('Desired',position)
        #print('Real',true_angle)
        return -true_angle

    def check_link_state(self):
        for lidarNb in range(constants.nb_of_lidars):
            try:
                self.lidars[lidarNb].get_health()  
            except:
                print('Link error with lidar',lidarNb+1)
                self.disconnect()
                return -1
            try:
                if self.serial_connection.ping(constants.servosIDs[lidarNb]) == False:
                    raise
            except:
                print('Link error with servo',lidarNb+1)
                self.disconnect()
                return -1
        return 0

    def stop_motors(self):
        for lidarNb in range(constants.nb_of_lidars):
            self.lidars[lidarNb].stop_motor()
        
    def start_motors(self):
        for lidarNb in range(constants.nb_of_lidars):
            self.lidars[lidarNb].start_motor()
            time.sleep(0.25) # To avoid to drain too much current
    
    def wait_servos_moving(self):
        for lidarNb in range(constants.nb_of_lidars):
            timeout = 100 # 10 seconds
            while True:
                try:
                    while self.serial_connection.is_moving(constants.servosIDs[lidarNb]) == True:
                        timeout -= 1
                        if timeout <= 0:
                            print("    Problem with the servo", lidarNb+1,"!")
                            self.disconnect()
                            return -1
                        time.sleep(0.1)
                    break
                except:
                    pass
        return 0
    
    def single_scan(self,current_angle_z,meas_nb,erase_file,height): 
        file = []
        iterMeas = []
        datasLeft = []
        done = []
        #print(meas_nb)
        for lidarNb in range(constants.nb_of_lidars):
            try:
                path = constants.dirPath + r'/DatasL' + str(lidarNb+1) + '.txt'
                if erase_file == True:
                    file.append(open(path,'w'))
                else:
                    file.append(open(path,'a'))
            except:
                print('    Cannot open file for lidar',lidarNb+1,'!')
                self.disconnect()
                return -1
            try:
                iterMeas.append(self.lidars[lidarNb].iter_measures())
            except:
                print('    Cannot communicate with lidar',lidarNb+1,'!')
                return -1 
            datasLeft.append(meas_nb)
            done.append(False)
            
        try:
            nb_datas_rot = -2 * np.ones(constants.nb_of_lidars)
            datas = next(iterMeas[lidarNb]) # Flush the first data
            while False in done:
                for lidarNb in range(constants.nb_of_lidars):
                    if done[lidarNb] == False:
                        datas = next(iterMeas[lidarNb])
                        angle = -1*(((datas[2]+constants.offset_angle_lidar)%360.0)-180.0)
                        dist = datas[3]
                        
                        if dist > 0:
                            
                        
                        if angle >= -90 and angle <= +90:
                            if nb_datas_rot[lidarNb] == -1:
                                nb_datas_rot[lidarNb] = 0
                                
                            if dist > 0:
                                # Get Pos
                                visionAngle = (constants.lidarsAngle[lidarNb] + 180)%360 # Get the opposite of the lidar's angle
                                c1 = math.cos(visionAngle*math.pi/180)
                                c2 = math.cos(current_angle_z*math.pi/180)
                                c3 = math.cos(angle*math.pi/180)
                                s1 = math.sin(visionAngle*math.pi/180)
                                s2 = math.sin(current_angle_z*math.pi/180)
                                s3 = math.sin(angle*math.pi/180)
                                x = dist*((c1*c2*c3)-(s1*s3)) + constants.lidarsDist*math.cos(constants.lidarsAngle[lidarNb]*math.pi/180)
                                y = dist*((c1*s3)+(c2*c3*s1)) + constants.lidarsDist*math.sin(constants.lidarsAngle[lidarNb]*math.pi/180)
                                z = dist*(-1)*(c3*s2) + constants.lidarsHeight
                                # If the data is in the detection circle
                                #if math.sqrt((x**2)+(y**2)) < (constants.lidarsDist-constants.deadZone):
                                #    if z >= height-constants.margin_bot and z <= height+constants.margin_top:
                                file[lidarNb].write(str(angle) + ' ' + str(current_angle_z) + ' ' + str(dist) + '\n')
                                if datasLeft[lidarNb] > 0:
                                    datasLeft[lidarNb] -= 1
                                if nb_datas_rot[lidarNb] >= 0:
                                    nb_datas_rot[lidarNb] += 1
                        else:
                            if nb_datas_rot[lidarNb] >= 0:
                                print("HELLO")
                                print(nb_datas_rot[lidarNb])
                            if datasLeft[lidarNb] == 0 or nb_datas_rot[lidarNb] == 0:
                                done[lidarNb] = True
                            nb_datas_rot[lidarNb] = -1
        except:
            print('    Cannot retrieve datas on lidar ',lidarNb+1,'!')
            self.disconnect()
            return -1
        
        for lidarNb in range(constants.nb_of_lidars):
            try:
                self.lidars[lidarNb].stop()
            except:
                print('    Cannot communicate with lidar',lidarNb+1,'!')
                return -1

            file[lidarNb].close()
        print(a)
        return math.ceil(np.mean(datasLeft))
    
    
    def scan_datas(self,height):
        print('Datas scanning...')
        self.start_motors()
        
        # Clean the infos file
        try:
            infoFile = open(constants.dirPath + r'/scan_infos.txt','w')
        except:
            print("    Cannot open infos file")
            return -1
        
        [inclination_array,nbMes_array] = compute_angle_array_for_scan(height)
        tmp_datas_left = 0
        for i in range(len(inclination_array)):
            try:
                # Broadcast moving action to all servos
                inclination_array[i] = self.servos_goto(BROADCAST_ID,inclination_array[i])
                if self.wait_servos_moving() != 0:
                    return -1
            except:
                print("    Problem with the serial connection!")
                self.disconnect()
                return -1

            nbMes_array[i] = math.ceil(nbMes_array[i]+tmp_datas_left)

            if i == 0:
                tmp_datas_left = self.single_scan(inclination_array[i],constants.nbOfDatasToRetrieve,True,height)
            else:
                tmp_datas_left = self.single_scan(inclination_array[i],constants.nbOfDatasToRetrieve,False,height)

            if tmp_datas_left == -1:
                return -1

            
        # Write the height of scan in the info file
        try:
            infoFile = open(constants.dirPath + r'/scan_infos.txt','a')
            infoFile.write(str(height))
        except:
            print("    Cannot open infos file")
            return -1
        
        try:
            # Broadcast moving action to all servos
            self.servos_goto(BROADCAST_ID,0)
        except:
            print("    Problem with the serial connection!")
            self.disconnect()
            return -1
    
        print('    All the datas have been successfully retrieved.') 
        self.stop_motors()
        return 0
        
