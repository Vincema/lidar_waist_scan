from rplidar import RPLidar
import constants 

# Documentation: http://rplidar.readthedocs.io/en/latest/
class driverLidars:
    def __init__(self):
        self.lidars = []
        self.areConnected = 0

    def connect(self):
        print('Connecting to lidars...')
        self.lidars = []
        for lidarNb in range(3):
            try:
                self.lidars.append(RPLidar(constants.serialPort[lidarNb],baudrate=115200))
                self.lidars[lidarNb].connect()
            except:
                print('    Cannot connect to the lidar ',lidarNb+1,'!')
                return -1

            print('    Connected to lidar ',lidarNb+1)
            self.areConnected = 1
            return 0
            
    def disconnect(self):
        print('Disconnecting to lidars...')
        for lidarNb in range(3):
            self.lidars[lidarNb].disconnect()
            print('    Disconnected to lidar',lidarNb+1)
        self.lidars[:] = []
        self.areConnected = 0

    def checkLinkState(self):
        for lidarNb in range(3):
            try:
                self.lidars[lidarNb].get_health()        
            except:
                print('Link problem with lidar',lidarNb+1)
                self.disconnect()
                return -1
        return 0

    def toggle_motors(self):
        for lidarNb in range(3):
            if self.lidars[lidarNb].motor == False:
                self.lidars[lidarNb].start_motor()
                self.lidars[lidarNb].motor = True
        else:
            self.lidars[lidarNb].stop_motor()
            self.lidars[lidarNb].motor = False
    
    def scan_datas(self):
        print('Datas scanning...')
        for lidarNb in range(3):
            self.lidars[lidarNb].start_motor()
            
        for lidarNb in range(3):
            try:
                path = constants.dirPath + r'DatasL' + str(lidarNb+1) + '.txt'
                file = open(path,'w')
                iterMeas = self.lidars[lidarNb].iter_measurments()
            except:
                print('    Cannot communicate/open file with/for the lidar ',lidarNb+1,'!')
                self.disconnect()
                return -1
                            
                datasLeft = constants.nbOfDatasToRetrieve
                while datasLeft > 0:
                    datasLeft -= 1
                    try:
                        datas = next(iterMeas)
                        angle = datas[2]
                        dist = datas[3]
                        file.write(str(angle-180) + ' ' + str(dist) + '\n')   
                    except:
                        print('    Cannot retrieve datas on lidar ',lidarNb+1,'!')
                        self.disconnect()
                        return -1

                self.lidars[lidarNb].stop()
                self.lidars[lidarNb].start_motor()
                self.lidars[lidarNb].clear_input()
                file.close()
                print('    Data retrieved on lidar ',lidarNb+1)
        return 0        
