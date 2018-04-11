
# Path to the datas directory
dirPath = r'/home/pi/Documents/lidar_waist_scan/lidars_datas'

# Angle of lidars
lidarsAngle = [11*180/6,
               7*180/6,
               3*180/6]

# Distance of lidars from center
lidarsDist = 250

# Height of lidars from the base
lidarsHeight = 1000

# Bounds of plots
boundsDatasLidars = [-400,
                     400,
                     -300,
                     400]

# Smooth Factor f optimization
fMin = 0.02
fMax = 0.15
stepNb = 15

# Nb of datas to retrieve for each lidar in total
nbOfDatasToRetrieve = 200

# Serial port for Lidars
serialPort = ['/dev/ttyUSB0',
              '/dev/ttyUSB1',
              '/dev/ttyUSB2']

# Speed of servos (0-255)
servosSpeed = 50

# Servos IDs
servosIDs = [1,2,3]

# Heights margin for scanning (arround the patient's height)
margin_top = 25
margin_bot = 25

# Size of the zone from the lidar where measurement can't be performed
deadZone = 100

# Clustering constants
deltaMinClust = 5  # in mm
aStartClust = 0    # Start value for dichotomy
bStartClust = 1000 # Start value for dichotomy
nonNoiseMeas = 0.9 # Minimum fraction of the measurments count considered as ok