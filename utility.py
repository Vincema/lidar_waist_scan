import math
import constants
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
import copy
from scipy.sparse.csgraph import minimum_spanning_tree
from scipy.spatial import Delaunay
from scipy.spatial import Voronoi,voronoi_plot_2d
from sklearn.neighbors import kneighbors_graph

# Misc data arrays
mergedPointsXY = []
centeredPointsXY = []
clusteredPointsXY = []
patientHeight = 0

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


def euclidian_dist(a,b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    
# Circle fitting
def fit_circle(points):
    n = len(points)
    x = points[:,0]
    y = points[:,1]
    
    # Least squares method
    A = n*np.sum(x**2) - np.sum(x)**2
    B = n*np.sum(x*y) - np.sum(x)*np.sum(y)
    C = n*np.sum(y**2) - np.sum(y)**2
    D = 0.5 * (n*np.sum(x*y**2) - np.sum(x)*np.sum(y**2) + n*np.sum(x**3) - np.sum(x**2)*np.sum(x))
    E = 0.5 * (n*np.sum(y*x**2) - np.sum(y)*np.sum(x**2) + n*np.sum(y**3) - np.sum(y**2)*np.sum(y))

    am = ((D*C)-(B*E)) / ((A*C) - B**2)
    bm = ((A*E)-(B*D)) / ((A*C) - B**2)
    origin = np.array([am,bm])
    radius = np.sum(np.sqrt((x-am)**2 + (y-bm)**2))/n

    return origin,radius


def is_useful_data(pointXYZ):
    # Is in the circle of 75% of the radius of the outter circle formed by the lidars
    if math.sqrt(pointXYZ.x**2 + pointXYZ.y**2) <= (constants.lidarsDist-constants.deadZone):
        # Is at the good height
        if pointXYZ.z >= patientHeight-constants.margin_bot and pointXYZ.z <= patientHeight+constants.margin_top:
            return True
    return False


def get_clusters(points,v1,v2):
    n_edges = len(v1)
    n = len(points)
    point_cluster = np.zeros(n)
    cluster_nb = 1
    for i in range(n_edges):
        ind1 = v1[i]
        ind2 = v2[i]
        if point_cluster[ind1] == 0 and point_cluster[ind2] == 0:
            point_cluster[ind1] = cluster_nb
            point_cluster[ind2] = cluster_nb
            cluster_nb += 1
        elif point_cluster[ind1] > 0 and point_cluster[ind2] > 0:
            if point_cluster[ind1] != point_cluster[ind2]:
                ind_to_replace = np.max([point_cluster[ind1],point_cluster[ind2]])
                ind_to_apply = np.min([point_cluster[ind1],point_cluster[ind2]])
                for j in range(n):
                    if point_cluster[j] == ind_to_replace:
                        point_cluster[j] = ind_to_apply
        elif point_cluster[ind1] > 0 and point_cluster[ind2] == 0:
            point_cluster[ind2] = point_cluster[ind1]
        elif point_cluster[ind2] > 0 and point_cluster[ind1] == 0:
            point_cluster[ind1] = point_cluster[ind2]

    n_clustered_pts = 0
    clusters,count = np.unique(point_cluster,return_counts=True)
    clust_pts = []
    # While not 75% of the points are clustered
    while n_clustered_pts/n < 0.75:
        ind_biggest_cluster = np.argmax(count)
        biggest_cluster = clusters[ind_biggest_cluster]
        ind = np.array([],'int')
        for i in range(n):
            if point_cluster[i] == biggest_cluster:
                ind = np.append(ind,i)
        clust_pts.append(points[ind])
        n_clustered_pts += len(ind)
        clusters = np.delete(clusters,ind_biggest_cluster)
        count = np.delete(count,ind_biggest_cluster)
    
    return clust_pts


# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    def ccw(A,B,C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def gabriel_graph(points):
    points = np.asarray(points)

    tri = Delaunay(points)
    vor = Voronoi(points)

    class ind_points_to_vertices:
        def __init__(self):
            self.dest_point = []
            self.vertice_index = []

    ind_points_to_vertices_arr = []
    for i in range(len(points)):
        ind_points_to_vertices_arr.append(ind_points_to_vertices())

    for i in range(len(vor.ridge_points)):
        ind1 = vor.ridge_points[i][0]
        ind2 = vor.ridge_points[i][1]
        if vor.ridge_vertices[i][0] >= 0 and vor.ridge_vertices[i][1] >= 0:
            ind_points_to_vertices_arr[ind1].dest_point.append(ind2)
            ind_points_to_vertices_arr[ind1].vertice_index.append(i)
            ind_points_to_vertices_arr[ind2].dest_point.append(ind1)
            ind_points_to_vertices_arr[ind2].vertice_index.append(i)

    ind_edges = []
    dist = np.array([])
    for i in range(len(tri.simplices)):
        for j in range(3):
            ind1 = tri.simplices[i][j]
            ind2 = tri.simplices[i][(j+1)%3]
            for k in range(len(ind_points_to_vertices_arr[ind1].dest_point)):
                if ind2 == ind_points_to_vertices_arr[ind1].dest_point[k]:
                    ind_vert = ind_points_to_vertices_arr[ind1].vertice_index[k]
                    a,b = vor.vertices[vor.ridge_vertices[ind_vert]]
                    if intersect(a,b,points[ind1],points[ind2]) == True:
                        ind_edges.append([ind1,ind2])
                        distance = euclidian_dist(points[ind1],points[ind2])
                        dist = np.append(dist,distance)
                    break
        

    ind_edges = np.asarray(ind_edges)
    #plt.plot([points[ind_edges[:,0],0],points[ind_edges[:,1],0]],[points[ind_edges[:,0],1],points[ind_edges[:,1],1]],'k')
    #voronoi_plot_2d(vor)
    #plt.show()
    return ind_edges,dist                


def remove_outliers():
    # Confidence interval of 99%
    dist_to_med_tresh = 3.09
    
    n = len(mergedPointsXY)
    points = np.zeros((n,2))
    for i in range(n):
        points[i,0] = mergedPointsXY[i].x
        points[i,1] = mergedPointsXY[i].y
    
    G = kneighbors_graph(points,n_neighbors=50,mode='distance')
    T = minimum_spanning_tree(G,overwrite=True)

    # Get coord of each line segment
    T = T.tocoo()
    dist = T.data
    p1 = T.row
    p2 = T.col
    A = points[p1].T
    B = points[p2].T
    x_coords = np.vstack([A[0], B[0]])
    y_coords = np.vstack([A[1], B[1]])

    # Rough filtering (if the edge is too long, delete it)
    n_edges = len(dist)
    median_dist = np.median(dist)
    std_dist = np.std(dist)
    dist_normal = (dist - median_dist) / std_dist
    
    ind = np.array([],'int')
    for i in range(n_edges):
        if dist_normal[i] < dist_to_med_tresh:
            ind = np.append(ind,i)

    p1 = p1[ind]
    p2 = p2[ind]
    clust_pts = get_clusters(points,p1,p2)

    # Fine filtering (if the edge is too long, delete it)
    global clusteredPointsXY
    clusteredPointsXY = []
    for k in range(len(clust_pts)):
        ind_edges,dist = gabriel_graph(clust_pts[k])
        
        n_edges = len(dist)
        median_dist = np.median(dist)
        std_dist = np.std(dist)
        dist_normal = (dist - median_dist) / std_dist
        
        ind = np.array([],'int')
        for i in range(n_edges):
            if dist_normal[i] < dist_to_med_tresh:
                ind = np.append(ind,i)
        ind_edges = ind_edges[ind]
        final_clust_pts = get_clusters(clust_pts[k],ind_edges[:,0],ind_edges[:,1])

        plt.figure(figMerge)
        for i in range(len(final_clust_pts)):
            for j in range(len(final_clust_pts[i])):
                clusteredPointsXY.append(point(final_clust_pts[i][j][0],final_clust_pts[i][j][1]))
                plt.plot(final_clust_pts[i][j][0],final_clust_pts[i][j][1],'.k',ms=2)





