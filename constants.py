
# Path to the datas directory
dirPath = r'/home/pi/Documents/lidar_waist_scan/lidars_datas/'

# Angle of lidars
lidarsAngle = [11*180/6,
               7*180/6,
               3*180/6]

# Distance of lidars from center
lidarsDist = [280,
              280,
              280]

# Bounds of plots
boundsDatasLidars = [-300,
                     300,
                     -200,
                     400]

# Smooth Factor f optimization
fMin = 0.02
fMax = 0.15
stepNb = 10

# Nb of datas to retrieve for each complete scan for each lidar
nbOfDatasToRetrieve = 2000

# Serial port for Lidars
serialPort = ['ttyUSB0',
              'ttyUSB1',
              'ttyUSB2']

