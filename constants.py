
# Path to the datas directory
dirPath = r'/home/pi/Documents/lidar_waist_scan/lidars_datas'

# Angle of lidars
lidarsAngle = [11*180/6,
               7*180/6,
               3*180/6]

# Distance of lidars from center
lidarsDist = 500

# Height of lidars from the base
lidarsHeight = 1000

# Bounds of plots
boundsDatasLidars = [-520,
                     520,
                     -400,
                     600]

# Smooth Factor f optimization
fMin = 0.02
fMax = 0.15
stepNb = 10

# Nb of datas to retrieve for each lidar in total
nbOfDatasToRetrieve = 2000

# Serial port for Lidars
serialPort = ['/dev/ttyUSB0',
              '/dev/ttyUSB1',
              '/dev/ttyUSB2']

# Speed of servos
servosSpeed = 50

# Servos IDs
servosIDs = [1,2,3]

# Scanning precision factor:
#   <1: Missing body part
#   =1: Scan the whole waist shape exactly
#   >1: Scan more than once some parts
#   =2: Scan twice the waist shape
scanPrecisionFactor = 1.5

# Heights margin for scanning (arround the patient's height)
margin_top = 25
margin_bot = 25

# Size of the zone from the lidar where measurement can't be performed
deadZone = 100
