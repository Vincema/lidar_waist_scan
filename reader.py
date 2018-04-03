import constants
import numpy as np

def read_data_single_lidar(lidarNb):
    path = constants.dirPath + r'\DatasL' + str(lidarNb+1) + '.txt'
    datas = np.loadtxt(path, dtype='d', delimiter=' ')
    return datas

