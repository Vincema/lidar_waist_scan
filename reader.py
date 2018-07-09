from bb_serial import cust_print, cust_read
import constants
import numpy as np

def read_data_lidar(lidarNb):
    try:    
        path = constants.dataPath + r'/DatasL' + str(lidarNb+1) + '.txt'
        data = np.loadtxt(path, dtype='d', delimiter=' ')
        return (data)
    except:
        cust_print('    Cannot read data file!')


def read_scan_infos():
    try:
        path = constants.dataPath + r'/scan_infos.txt'
        height = np.loadtxt(path)
        return (height)
    except:
        cust_print('    Cannot read info file!')
