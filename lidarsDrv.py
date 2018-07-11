from bb_serial import cust_print, cust_read
from drivers.rplidar import RPLidar
from pyax12.connection import Connection
import numpy as np
import constants
import time
import math

BROADCAST_ID = 254

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
    max_meas_dist = 2*(constants.lidarsDist-constants.deadZone) + constants.deadZone
    
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
        meas_nb.append(constants.nbOfDatasToRetrieve)
        #if dist2 > max_meas_dist:
        #    meas_nb.append(math.ceil(constants.nbOfDatasToRetrieve * (max_meas_dist-dist1) / (max_meas_dist-min_meas_dist)))
        #else:
        #    meas_nb.append(math.ceil(constants.nbOfDatasToRetrieve * (dist2-dist1) / (max_meas_dist-min_meas_dist)))
    return [angle_array,meas_nb]
    

# Documentation: http://rplidar.readthedocs.io/en/latest/
class driverLidars:
    def __init__(self):
        self.lidars = []
        self.serial_connection = []
        self.areConnected = 0

    def connect(self):
        cust_print('Connecting to lidars and servos...')
        self.lidars = []
        self.serial_connection = []
        try:
            # Connect to the serial port
            self.serial_connection = Connection(port="/dev/ttyAMA0", baudrate=57600, rpi_gpio=True)
        except:
            cust_print('    Cannot open serial connection for servos!')
            #return -1
        for lidarNb in range(constants.nb_of_lidars):
            try:
                self.lidars.append(RPLidar(constants.serialPort[lidarNb],baudrate=115200))
                self.lidars[lidarNb].connect()
            except:
                cust_print('    Cannot connect to the lidar' + str(lidarNb+1) + '!')
                return -1
            cust_print('    Connected to lidar' + str(lidarNb+1))
            
            try:
                # Try to ping the motor
                if self.serial_connection.ping(constants.servosIDs[lidarNb]) == False:
                    raise
                self.serial_connection.set_speed(BROADCAST_ID,constants.servosSpeed)
                self.servos_goto(constants.servosIDs[lidarNb],0)
            except:
                cust_print('    Cannot connect to the servo' + str(lidarNb+1) + '!')
                return -1
            cust_print('    Connected to servo' + str(lidarNb+1))
            time.sleep(0.25) # To avoid a too high current drain
        self.areConnected = 1
        return 0
        
    def disconnect(self):
        cust_print('Disconnecting to lidars and servos...')
        self.serial_connection.close()
        cust_print('    Disconnected to servos serial')
        for lidarNb in range(constants.nb_of_lidars):
            self.lidars[lidarNb].stop()
            self.lidars[lidarNb].disconnect()
            time.sleep(0.25) # To avoid to drain too much current
            cust_print('    Disconnected to lidar' + str(lidarNb+1))
        self.lidars[:] = []
        self.areConnected = 0

    def servos_goto(self,servosID,position):   
        self.serial_connection.goto(servosID,position,degrees=True)

    def check_link_state(self):
        for lidarNb in range(constants.nb_of_lidars):
            try:
                self.lidars[lidarNb].get_health()  
            except:
                cust_print('Link error with lidar' + str(lidarNb+1))
                self.disconnect()
                return -1
            try:
                if self.serial_connection.ping(constants.servosIDs[lidarNb]) == False:
                    raise
            except:
                cust_print('Link error with servo' + str(lidarNb+1))
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
                            cust_print("    Problem with the servo" + str(lidarNb+1) + "!")
                            self.disconnect()
                            return -1
                        time.sleep(0.1)
                    break
                except:
                    pass
        return 0
    
    def single_scan(self,current_angle_z,meas_nb,erase_file): 
        file = []
        iterMeas = []
        datasLeft = []
        done = []
        for lidarNb in range(constants.nb_of_lidars):
            try:
                path = constants.dirPath + r'/DatasL' + str(lidarNb+1) + '.txt'
                if erase_file == True:
                    file.append(open(path,'w'))
                else:
                    file.append(open(path,'a'))
            except:
                cust_print('    Cannot open file for lidar' + str(lidarNb+1) + '!')
                self.disconnect()
                return -1
            try:
                iterMeas.append(self.lidars[lidarNb].iter_measures())
            except:
                cust_print('    Cannot communicate with lidar' + str(lidarNb+1) + '!')
                return -1 
            datasLeft.append(constants.nbOfDatasToRetrieve)
            done.append(False)
            
        try:  
            while False in done:
                for lidarNb in range(constants.nb_of_lidars):
                    if done[lidarNb] == False:
                        datas = next(iterMeas[lidarNb])
                        angle = -1*((datas[2]+constants.offset_angle_lidar)%360 - 180.0)
                        dist = datas[3]
                        # First selection of points 
                        if angle >= -90 and angle <= +90 and dist > 0:
                            file[lidarNb].write(str(angle) + ' ' + str(-current_angle_z) + ' ' + str(dist) + '\n')
                            datasLeft[lidarNb] -= 1
                            if datasLeft[lidarNb] < 1:
                                done[lidarNb] = True
        except:
            cust_print('    Cannot retrieve datas on lidar ' + str(lidarNb+1) + '!')
            self.disconnect()
            return -1
        
        for lidarNb in range(constants.nb_of_lidars):
            try:
                self.lidars[lidarNb].stop()
            except:
                cust_print('    Cannot communicate with lidar'+ str(lidarNb+1) + '!')
                return -1

            file[lidarNb].close()
        return 0
    
    
    def scan_datas(self,stature):
        cust_print('Datas scanning...')

        # Get height from stature
        lin_coeffs_navel_height = [0.66410011, -105.89395578]
        height = (stature*10)*lin_coeffs_navel_height[0] + lin_coeffs_navel_height[1]

        self.start_motors()
        
        # Clean the infos file
        try:
            infoFile = open(constants.dirPath + r'/scan_infos.txt','w')
        except:
            cust_print("    Cannot open infos file")
            return -1
        
        [inclination_array,nbMes_array] = compute_angle_array_for_scan(height)
        
        for i in range(len(inclination_array)):
            try:
                # Broadcast moving action to all servos
                self.servos_goto(BROADCAST_ID,inclination_array[i])
            except:
                cust_print("    Problem with the serial connection!")
                self.disconnect()
                return -1
            
            if self.wait_servos_moving() != 0:
                return -1

            if i == 0:
                if self.single_scan(inclination_array[i],nbMes_array[i],True) != 0:
                    return -1
            else:
                if self.single_scan(inclination_array[i],nbMes_array[i],False) != 0:
                    return -1                    
            
        # Write the height of scan in the info file
        try:
            infoFile = open(constants.dirPath + r'/scan_infos.txt','a')
            infoFile.write(str(height))
        except:
            cust_print("    Cannot open infos file")
            return -1
        
        try:
            # Broadcast moving action to all servos
            self.servos_goto(BROADCAST_ID,0)
        except:
            cust_print("    Problem with the serial connection!")
            self.disconnect()
            return -1
    
        cust_print('    All the datas have been successfully retrieved.') 
        self.stop_motors()
        return 0
        
