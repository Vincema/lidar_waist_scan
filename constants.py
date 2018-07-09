import math
import os

#### DEBUG ####
# Number of lidars that should be connected
nb_of_lidars = 1
use_bb_serial = False
disp_charts = True

# Path to the datas directory
dataPath = os.getcwd() + '\data'

lidarsAngle = [11*180/6,
               7*180/6,
               3*180/6]

# Distance of lidars from center
lidarsDist = 1000

# Size of an edge of the equi triangle
edgeLen = 3*lidarsDist/math.sqrt(3)

# Distance lidar from vertical rotation axis (servo)
lidarsDistFromRotAxis = 50

# Height of lidars from the base
lidarsHeight = 1000

# Size of the zone from the lidar where measurement can't be performed (mm)
deadZone = 300

# Bounds of plots
boundsDatasLidars = [-600,
                     600,
                     -600,
                     600]

# Nb of datas to retrieve for each lidar in total
nbOfDatasToRetrieve = 100

# Offset angle lidars
offset_angle_lidar = 180

# Serial port for Lidars
serialPort = ['/dev/ttyUSB0',
              '/dev/ttyUSB3',
              '/dev/ttyUSB1']

# Speed of servos (0-254)
servosSpeed = 50

# Servos IDs
servosIDs = [1,2,4]

# Heights margin for scanning (arround the patient's height)
margin_top_meas = 10
margin_bot_meas = 10
margin_top = 20
margin_bot = 20

# Clusterung
min_size_cluster = 10
max_dist_bet_clusters = 1000

# Bitbang serial
bb_serial_RX = 27
bb_serial_TX = 17
bb_baudrate = 19200

