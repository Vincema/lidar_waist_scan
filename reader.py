import constants
import numpy as np

def read_data_single_lidar(lidarNb):
    try:    
        path = constants.dirPath + r'/DatasL' + str(lidarNb+1) + '.txt'
        datas = np.loadtxt(path, dtype='d', delimiter=' ')
        path = constants.dirPath + r'/scan_infos.txt'
        height = np.loadtxt(path)
    except:
        print('    Cannot read data files!')
    return (datas,height)

