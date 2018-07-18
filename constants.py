import math
import os

#### DEBUG ####
# Number of lidars that should be connected
nb_of_lidars = 2
use_bb_serial = False
disp_charts = True

# Path to the datas directory
dataPath = r'/home/pi/Documents/lidar_waist_scan/data'

lidarsAngle = [11*180/6,
               7*180/6,
               3*180/6]

# Distance of lidars from center
lidarsDist = 540

# Size of an edge of the equi triangle
edgeLen = 3*lidarsDist/math.sqrt(3)

# Distance lidar from vertical rotation axis (servo)
lidarsDistFromRotAxis = 25

# Height of lidars from the base
lidarsHeight = 900

# Size of the zone from the lidar where measurement can't be performed (mm)
deadZone = 200

# Bounds of plots
boundsDatasLidars = [-600,
                     600,
                     -600,
                     600]

# Nb of datas to retrieve for each lidar for each angle
nbOfDatasToRetrieve = 500

# Offset angle lidars
offset_angle_lidar = 0

# Serial port for Lidars
serialPort = ['/dev/ttyUSB1',
              '/dev/ttyUSB0',
              '/dev/ttyUSB2']

# Speed of servos (0-254)
servosSpeed = 30

# Servos IDs
servosIDs = [1,2,3]

# Minimum points before computation
min_points = 25

# Heights margin for scanning (arround the patient's height)
margin_top_meas = 10
margin_bot_meas = 10
margin_top = 10
margin_bot = 10

# Bitbang serial
bb_serial_RX = 27
bb_serial_TX = 17
bb_baudrate = 19200

