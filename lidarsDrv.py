from rplidar import RPLidar
from pyax12.connection import Connection
import constants
import time
import math

def compute_angle_array_for_scan(height):
    angle_array = []
    meas_nb = []
    
    # If the patient's height is at the lidar level, no need to change angle 
    if abs(constants.lidarsHeight - height) <= constants.margin_top or abs(constants.lidarsHeight - height) <= constants.margin_bot:
        angle_array.append(0)
        meas_nb.append(constants.nbOfDatasToRetrieve)
        return [angle_array,meas_nb]
    
    hgt_diff_mtop = constants.lidarsHeight - (height + constants.margin_top)
    hgt_diff_mbot = constants.lidarsHeight - (height - constants.margin_bot)
    
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
        currentDist += (dist2-dist1)/constants.scanPrecisionFactor
        angle_array.append(angle*180/math.pi)
        meas_nb.append(math.ceil(constants.nbOfDatasToRetrieve * ((dist2-dist1)/(max_meas_dist-min_meas_dist)) / constants.scanPrecisionFactor))
    
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
        for lidarNb in range(3):
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
                self.serial_connection.set_speed(constants.servosIDs[lidarNb],constants.servosSpeed)
                self.serial_connection.goto(constants.servosIDs[lidarNb],0,degrees=True)
            except:
                print('    Cannot connect to the servo',lidarNb+1,'!')
                return -1
            print('    Connected to servo',lidarNb+1)
            time.sleep(0.25) # To avoid too much drained current
        self.areConnected = 1
        return 0
        
    def disconnect(self):
        print('Disconnecting to lidars and servos...')
        self.serial_connection.close()
        print('    Disconnected to servos serial')
        for lidarNb in range(3):
            self.lidars[lidarNb].disconnect()
            time.sleep(0.25) # To avoid to drain too much current
            print('    Disconnected to lidar',lidarNb+1)
        self.lidars[:] = []
        self.areConnected = 0

    def check_link_state(self):
        for lidarNb in range(3):
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
        for lidarNb in range(3):
            self.lidars[lidarNb].stop_motor()
        
    def start_motors(self):
        for lidarNb in range(3):
            self.lidars[lidarNb].start_motor()
            time.sleep(0.25) # To avoid to drain too much current
    
    def wait_servo_moving(self,lidarNb):
        timeout = 300 # 3 seconds
        try:
            while self.serial_connection.is_moving(constants.servosIDs[lidarNb]) == True:
                timeout -= 1
                if timeout == 0:
                    raise
                time.sleep(0.01)
        except:
            print("    Problem with the servo", lidarNb+1,"!")
            return -1
        return 0
    
    def single_scan(self,meas_nb,current_angle):
        for lidarNb in range(3):
            try:
                path = constants.dirPath + r'/DatasL' + str(lidarNb+1) + '.txt'
                file = open(path,'w')
            except:
                print('    Cannot open file for lidar',lidarNb+1,'!')
                self.disconnect()
                return -1
            
            try:
                iterMeas = self.lidars[lidarNb].iter_measurments()    
                datasLeft = meas_nb
                while datasLeft > 0:
                    datasLeft -= 1
                    datas = next(iterMeas)
                    angle = datas[2]
                    dist = datas[3]
                    file.write(str(angle-180) + ' ' + str(dist) + ' ' + str(current_angle) + '\n')   
            except:
                print('    Cannot retrieve datas on lidar ',lidarNb+1,'!')
                self.disconnect()
                return -1

            self.lidars[lidarNb].stop()
            self.lidars[lidarNb].clear_input()
            file.close()
        return 0
    
    def scan_datas(self,height):
        print('Datas scanning...')
        self.start_motors()
        
        if self.single_scan(200,0) != 0:
            return -1
        
        self.stop_motors()
        return 0
        