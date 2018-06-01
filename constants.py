import math

#### DEBUG ####
# Number of lidars that should be connected
nb_of_lidars = 1

# Path to the datas directory
dirPath = r'/home/pi/Documents/lidar_waist_scan/lidars_datas'

lidarsAngle = [11*180/6,
               7*180/6,
               3*180/6]

# Distance of lidars from center
lidarsDist = 500

# Size of an edge of the equi triangle
edgeLen = 3*lidarsDist/math.sqrt(3)

# Height of lidars from the base
lidarsHeight = 1000

# Size of the zone from the lidar where measurement can't be performed (mm)
deadZone = 200

# Bounds of plots
boundsDatasLidars = [-600,
                     600,
                     -600,
                     600]

# Nb of datas to retrieve for each lidar in total
nbOfDatasToRetrieve = 500

# Offset angle lidars
offset_angle_lidar = 180

# Serial port for Lidars
serialPort = ['/dev/ttyUSB0',
              '/dev/ttyUSB1',
              '/dev/ttyUSB2']

# Speed of servos (0-254)
servosSpeed = 50

# Servos IDs
servosIDs = [1,2,4]

# Servos limit angle
limit_ccw_angle = 0
limit_cw_angle = 90

# Heights margin for scanning (arround the patient's height)
margin_top_meas = 10
margin_bot_meas = 10
margin_top = 10
margin_bot = 10

# Clusterung
min_size_cluster = 10
max_dist_bet_clusters = 1000
