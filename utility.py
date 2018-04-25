import math
import constants
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial
from scipy import cluster


# Misc data arrays
mergedPointsXY = []
clusteredPointsXY = []
centeredPointsXY = []
distancePoints = []
loessPointsIter1 = []
loessPointsIter2 = []
loessResultXY = []
patientHeight = 0

# Misc variables
circum = 0

# Figures
figRaw = 1
figMerge = 2


class point:
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z


class distAngleData:
    def __init__(self,dist=0,alpha=0):
        self.dist = dist
        self.alpha = alpha


def is_useful_data(pointXYZ):
    # Is in the circle of 75% of the radius of the outter circle formed by the lidars 
    if math.sqrt(pointXYZ.x**2 + pointXYZ.y**2) <= (constants.lidarsDist-constants.deadZone):
        # Is at the good height
        if pointXYZ.z >= patientHeight-constants.margin_bot and pointXYZ.z <= patientHeight+constants.margin_top:
            return True
    return False


def compute_circumference():
    global loessResultXY
    global circum

    circum = 0
    loessResultXY
    for i in range(len(loessResultXY)-1):
        distX = loessResultXY[i+1].x - loessResultXY[i].x
        distY = loessResultXY[i+1].y - loessResultXY[i].y
        circum += math.sqrt(distX**2 + distY**2)
    print('\nCircumference: ',format(circum, '.2f'),'mm')


def get_neighbor_vertices(tri,vertNb):
    n1 = tri.vertex_neighbor_vertices[0][vertNb]
    n2 = tri.vertex_neighbor_vertices[0][vertNb+1]
    return tri.vertex_neighbor_vertices[1][n1:n2]


def average_sampling_radius(points):
    n = len(points)
    tri_points = spatial.Delaunay(points)
    
    # Lenght of the max edge lenght for each vertex 
    max_dist_vertices = []
    d_max = []
    for i in range(n):
        neighbors = get_neighbor_vertices(tri_points,i)
        dist_neighbors = []
        for j in neighbors:
            dist = math.sqrt((points[i][0]-points[j][0])**2 + (points[i][1]-points[j][1])**2)
            dist_neighbors.append(dist)
        d_max.append(np.max(dist_neighbors))

    return (np.sum(d_max)/n)
    

class unit:
    def __init__(self,coordinates,size,i,j):
        self.coord = coordinates
        self.size = size
        self.i = i
        self.j = j
        self.points = []
        self.pts_cnt = 0
        self.handled = False
        
    def add_point(self,point):
        self.pts_cnt += 1
        self.points.append(point)
                

class subgrid:
    def __init__(self,grid,imin,imax,jmin,jmax):
        self.imin = imin
        self.imax = imax
        self.jmin = jmin
        self.jmax = jmax
        self.bound_open = []
        self.barycenter = []
        self.bound_open = [True,True,True,True]
        self.set_bound_close(grid)
        self.locked_bound = ''
    
    def extend(self,bound,grid,cnt=1):
        if bound == 'l':
            self.imin -= int(np.floor(cnt))
        elif bound == 'r':
            self.imax += int(np.floor(cnt))
        elif bound == 'u':
            self.jmax += int(np.floor(cnt))
        elif bound == 'd':
            self.jmin -= int(np.floor(cnt))
        self.set_bound_close(grid)
        
    def retract(self,bound,grid,cnt=1):
        if bound == 'l':
            self.imin += int(np.floor(cnt))
        elif bound == 'r':
            self.imax -= int(np.floor(cnt))
        elif bound == 'u':
            self.jmax -= int(np.floor(cnt))
        elif bound == 'd':
            self.jmin += int(np.floor(cnt))
        self.set_bound_close(grid)
    
    def pts_cnt_subgrid(self,grid,count_handled=True):
        pts_cnt = 0
        for i in range(self.imin,self.imax+1):
            for j in range(self.jmin,self.jmax+1):
                if (not count_handled and not grid.units[i][j].handled) or count_handled:
                    pts_cnt += grid.units[i][j].pts_cnt
        return pts_cnt
        
    def set_bound_close(self,grid):
        # UP
        self.bound_open[0] = False
        for i in range(self.imin,self.imax+1):
            if not grid.units[i][self.jmax].handled and grid.units[i][self.jmax].pts_cnt > 0:
                self.bound_open[0] = True

        # DOWN
        self.bound_open[1] = False
        for i in range(self.imin,self.imax+1):
            if not grid.units[i][self.jmin].handled and grid.units[i][self.jmin].pts_cnt > 0:
                self.bound_open[1] = True

        # LEFT
        self.bound_open[2] = False
        for j in range(self.jmin,self.jmax+1):
            if not grid.units[self.imin][j].handled and grid.units[self.imin][j].pts_cnt > 0:
                self.bound_open[2] = True

        # RIGHT
        self.bound_open[3] = False
        for j in range(self.jmin,self.jmax+1):
            if not grid.units[self.imax][j].handled and grid.units[self.imax][j].pts_cnt > 0:
                self.bound_open[3] = True
    
    def compute_barycenter(self,grid):
        pts_cnt = 0
        pts = []
        for i in range(self.imin,self.imax+1):
            for j in range(self.jmin,self.jmax+1):
                pts_cnt += grid.units[i][j].pts_cnt
                for k in range(grid.units[i][j].pts_cnt):
                    pts.append(grid.units[i][j].points[k])
        self.barycenter = barycenter(pts,pts_cnt)
        
    def set_handled_datas(self,grid):
        for i in range(self.imin,self.imax+1):
            for j in range(self.jmin,self.jmax+1):
                grid.units[i][j].handled = True
    
    def getting_bigger(self,grid):
        if self.bound_open[0] and self.locked_bound != 'u':
            self.extend('u',grid)
        if self.bound_open[1] and self.locked_bound != 'd':
            self.extend('d',grid)
        if self.bound_open[2] and self.locked_bound != 'l':
            self.extend('l',grid)
        if self.bound_open[3] and self.locked_bound != 'r':
            self.extend('r',grid)   
        
    def first_join(self,grid):
        stop = False
        while stop == False:
            self.getting_bigger(grid)
            
            stop = False
            closed_bounds = 0
            for i in self.bound_open:
                if i == False:
                    closed_bounds += 1
            if closed_bounds >= 2:
                stop = True

            bounds_lenght = [self.imax - self.imin + 1, self.jmax - self.jmin + 1]
            longest_bound_lenght = np.max(bounds_lenght)
            if longest_bound_lenght > 0:
                stop = True
                
            self.getting_bigger(grid)
            
        self.compute_barycenter(grid)
        
    def join_cycle(self,grid,width):
        while True:            
            stopW = False
            bounds_lenght = [(self.imax - self.imin + 1)*grid.units[0][0].size, (self.jmax - self.jmin + 1)*grid.units[0][0].size]
            longest_bound_lenght = np.max(bounds_lenght)
            if longest_bound_lenght > width:
                stopW = True

            stopU = not self.bound_open[0]
            stopD = not self.bound_open[1]
            stopL = not self.bound_open[2]
            stopR = not self.bound_open[3]
            if (stopU and stopD) or (stopL and stopR) or stopW:
                break
                
            self.getting_bigger(grid)

            # Stop condition
            closed_bounds = 0
            for i in self.bound_open:
                if i == False:
                    closed_bounds += 1
            if closed_bounds == 4:
                return True
        
        self.compute_barycenter(grid)
        self.set_handled_datas(grid)
        return False

    def retract_until_all_bounds_open(self,grid):
        while False in self.bound_open:
            for i in range(4):
                if i == 0 and not self.bound_open[0]:
                    self.retract('u',grid)
                if i == 1 and not self.bound_open[1]:
                    self.retract('d',grid)
                if i == 2 and not self.bound_open[2]:
                    self.retract('l',grid)
                if i == 3 and not self.bound_open[3]:
                    self.retract('r',grid)

    
    def get_neighbor(self,grid):
        subg_width = self.imax - self.imin + 1
        subg_height = self.jmax - self.jmin + 1

        neigh = None
        maximum = 0
        # UP
        if self.locked_bound != 'u':
            subgU = subgrid(grid, self.imin, self.imax, self.jmax+1, self.jmax+subg_height)
            cnt = subgU.pts_cnt_subgrid(grid,count_handled=True)
            if cnt > maximum:
                maximum = cnt
                if subgU.pts_cnt_subgrid(grid,count_handled=False) != 0:
                    subgU.locked_bound = 'd'
                    subgU.retract_until_all_bounds_open(grid)
                    l1 = subgU.imax - subgU.imin + 1
                    l2 = subgU.jmax - subgU.jmin + 1
                    subgU.retract('u',grid,l2/2)
                    subgU.retract('l',grid,l1/4)
                    subgU.retract('r',grid,l1/4)
                    neigh = subgU

        # DOWN
        if self.locked_bound != 'd':    
            subgD = subgrid(grid, self.imin, self.imax, self.jmin-subg_height, self.jmin-1)
            cnt = subgD.pts_cnt_subgrid(grid,count_handled=True)
            if cnt > maximum:
                maximum = cnt
                if subgD.pts_cnt_subgrid(grid,count_handled=False) != 0:
                    subgD.locked_bound = 'u'
                    subgD.retract_until_all_bounds_open(grid)
                    l1 = subgD.imax - subgD.imin + 1
                    l2 = subgD.jmax - subgD.jmin + 1
                    subgD.retract('d',grid,l2/2)
                    subgD.retract('l',grid,l1/4)
                    subgD.retract('r',grid,l1/4)
                    neigh = subgD

        # LEFT
        if self.locked_bound != 'l': 
            subgL = subgrid(grid, self.imin-subg_width, self.imin-1, self.jmin, self.jmax)
            cnt = subgL.pts_cnt_subgrid(grid,count_handled=True)
            if cnt > maximum:
                maximum = cnt
                if subgL.pts_cnt_subgrid(grid,count_handled=False) != 0:
                    subgL.locked_bound = 'r'
                    subgL.retract_until_all_bounds_open(grid)
                    l1 = subgL.jmax - subgL.jmin + 1
                    l2 = subgL.imax - subgL.imin + 1
                    subgL.retract('l',grid,l2/2)
                    subgL.retract('u',grid,l1/4)
                    subgL.retract('d',grid,l1/4)
                    neigh = subgL

        # RIGHT
        if self.locked_bound != 'r': 
            subgR = subgrid(grid, self.imax+1, self.imax+subg_width, self.jmin, self.jmax)
            cnt = subgR.pts_cnt_subgrid(grid,count_handled=True)
            if cnt > maximum:
                maximum = cnt
                if subgR.pts_cnt_subgrid(grid,count_handled=False) != 0:
                    subgR.locked_bound = 'l'
                    subgR.retract_until_all_bounds_open(grid)
                    l1 = subgR.jmax - subgR.jmin + 1
                    l2 = subgR.imax - subgR.imin + 1
                    subgR.retract('r',grid,l2/2)
                    subgR.retract('u',grid,l1/4)
                    subgR.retract('d',grid,l1/4)
                    neigh = subgR

        return neigh
    
        
class global_grid:
    def __init__(self,asr,points):
        self.global_grid = []
        self.asr = asr
        self.units = []
        
        grid_size = 2*(constants.lidarsDist + (asr-(constants.lidarsDist%asr)))
        nb_unit_per_row = int(grid_size/asr)

        self.imin = 0
        self.imax = -1
        self.jmin = 0
        self.jmax = -1
    
        x_pos = -grid_size/2 + asr/2
        for i in range(nb_unit_per_row):
            self.imax += 1
            row = []
            self.jmax = -1
            y_pos = -grid_size/2 + asr/2
            for j in range(nb_unit_per_row):
                self.jmax += 1
                row.append(unit([x_pos,y_pos],asr,i,j))
                y_pos += asr
            self.units.append(row)
            x_pos += asr

        origin = [self.units[self.imin][self.jmin].coord[0]-asr/2, self.units[self.imin][self.jmin].coord[1]-asr/2] 
        for i in range(len(points)):
            temp_i = int(np.floor(((points[i][0]-origin[0])/grid_size) * (self.imax - self.imin + 1))) + self.imin
            temp_j = int(np.floor(((points[i][1]-origin[1])/grid_size) * (self.jmax - self.jmin + 1))) + self.jmin
            self.units[temp_i][temp_j].add_point(points[i])
                    
   
        
def barycenter(points,n=None):
    if n == None:
        n = len(points)
    sum_x = np.sum(list(points[i][0] for i in range(n)))
    sum_y = np.sum(list(points[i][1] for i in range(n)))
    return [sum_x/n,sum_y/n]


# Globalwidth at P2 for the subgrid 2
def compute_global_width(grid,subg1,subg2):
    P1 = subg1.barycenter
    P2 = subg2.barycenter
    T = [P2[0]-P1[0], P2[1]-P2[1]]
    L = [-T[1], T[0]]
    norm_L = math.sqrt(L[0]**2 + L[1]**2)
    
    proj_dist = []
    for i in range(subg2.imin,subg2.imax+1):
        for j in range(subg2.jmin,subg2.jmax+1):
            for k in range(grid.units[i][j].pts_cnt):
                pt_vect = [grid.units[i][j].points[k][0]-P2[0],grid.units[i][j].points[k][1]-P2[1]]
                dist = (pt_vect[0] * L[0]/norm_L) + (pt_vect[1] * L[1]/norm_L)
                proj_dist.append([dist,i,j,k])

    if len(proj_dist) == 1:
        return math.inf

    maximum = 0
    minimum = math.inf
    
    for m in range(len(proj_dist)):
        if proj_dist[m][0] > maximum:
            maximum = proj_dist[m][0]
            max_i = int(proj_dist[m][1])
            max_j = int(proj_dist[m][2])
            max_k = int(proj_dist[m][3])
        if proj_dist[m][0] < minimum:
            minimum = proj_dist[m][0]
            min_i = int(proj_dist[m][1])
            min_j = int(proj_dist[m][2])
            min_k = int(proj_dist[m][3])

    width = math.sqrt((grid.units[max_i][max_j].points[max_k][0] - grid.units[min_i][min_j].points[min_k][0])**2
                    + (grid.units[max_i][max_j].points[max_k][1] - grid.units[min_i][min_j].points[min_k][1])**2)
    return width


"""
# Clusterize the datas points until all points are clusterized
def clustering(points):
    n = len(points)
    
    tree = cluster.hierarchy.linkage(points,method='centroid',metric='euclidean')
    clusters = list([x] for x in range(n))
    tree_iter = 0
    
    # While all points are not in a cluster
    done = False
    while done == False:
        ind0,ind1,dit,nbValues = tree[tree_iter]
        ind0 = int(ind0)
        ind1 = int(ind1)
        tmp_pts0 = clusters[ind0]
        tmp_pts1 = clusters[ind1]
        clusters.append(clusters[ind0] + clusters[ind1])
        clusters[ind0] = []
        clusters[ind1] = []
        tree_iter += 1
        
        done = True
        for i in range(n):
            if clusters[i] != []:
                done = False
    
    tmp_clust = clusters
    clusters = []
    clust_coordinates = []
    for i in range(len(tmp_clust)):
        if tmp_clust[i] != []:
            clusters.append(tmp_clust[i])
            tmp_pts = []
            for j in range(len(tmp_clust[i])):
                tmp_pts.append(tmp_clust[i][j])
            clust_coordinates.append(barycenter(list(points[i] for i in tmp_pts)))    
    
    print(clusters)
    print(clust_coordinates)
"""

def joining_scheme(grid,points):
    # Find the fisrt feature unit (minimum i)
    stop = False
    for i in range(grid.imin,grid.imax+1):
        if stop == True:
            break
        else:
            for j in range(grid.jmin,grid.jmax+1):
                if grid.units[i][j].pts_cnt > 0:   
                    first_subg = subgrid(grid,i,i,j,j)
                    stop = True
                    break

    subg_set = []
    try:
        first_subg.first_join(grid)
        #subg_set.append(first_subg)
        sec_subg = first_subg.get_neighbor(grid)
        sec_subg.first_join(grid)
        #subg_set.append(sec_subg)
        width = compute_global_width(grid,first_subg,sec_subg)
        print(width,str(sec_subg.pts_cnt_subgrid(grid)))
        prev_subg = sec_subg

        while True:
            subg = prev_subg.get_neighbor(grid)
            if subg == None:
                break
            if subg.join_cycle(grid,width):
                subg_set.append(subg)
                break
            width = compute_global_width(grid,prev_subg,subg)
            print(width,str(subg.pts_cnt_subgrid(grid)))
            subg_set.append(subg)
            prev_subg = subg
    except:
        print("Error during join algorithm!")
        pass

    fig = plt.figure(figMerge)
    ax1 = fig.add_subplot(111)
    for i in subg_set:
        ax1.add_patch(matplotlib.patches.Rectangle((grid.units[i.imin][i.jmin].coord[0] - grid.asr/2,
                                                    grid.units[i.imin][i.jmin].coord[1] - grid.asr/2),
                                                    grid.units[i.imax][i.jmax].coord[0] - grid.units[i.imin][i.jmin].coord[0] + grid.asr,
                                                    grid.units[i.imax][i.jmax].coord[1] - grid.units[i.imin][i.jmin].coord[1] + grid.asr,
                                                    fill= False))
        #ax1.text(grid.units[i.imin][i.jmin].coord[0] - grid.asr/2, grid.units[i.imin][i.jmin].coord[1] - grid.asr/2, str(i.pts_cnt_subgrid(grid)))
    return subg_set


def determine_order(points):
    asr = average_sampling_radius(points)
    grid = global_grid(asr*2,points)
    subgrids = joining_scheme(grid,points)
    return subgrids
    

def contour():
    global mergedPointsXY
    datas = mergedPointsXY
    points = []
    for i in range(len(datas)):
        points.append([datas[i].x,datas[i].y])

    subgrids = determine_order(points)

    barx = []
    bary = []
    for s in subgrids:
        barx.append(s.barycenter[0])
        barx.append(s.barycenter[1])
    plt.plot(s.barycenter[1],'kx')
    
    

