from bb_serial import cust_print, cust_read
import utility
import constants
import math
import numpy as np
import matplotlib.pyplot as plt
import copy
from scipy.interpolate import BSpline, splprep,splev
from scipy.linalg import solve,lstsq
from scipy.spatial import distance
import time

NB_OF_CTRL_POINTS = 10
ORDER = 3
NB_POINTS_BSPL = 50*NB_OF_CTRL_POINTS
APPROXIMATION_ERROR_THRESHOLD = 1.0
ZERO_LINE_THRESHOLD = 0.1
ITER_MAX = 10
REGUL_WEIGHT = 0.1


class bspline:
    def __init__(self,ctrl_pts):
        x = ctrl_pts[:,0]
        y = ctrl_pts[:,1]
        self.n_c = len(x)
        self.t = range(-ORDER, ORDER+self.n_c+1)
        self.t_max = self.n_c 
        self.t_min = 0
        self.c = np.array([x,y])
        self.update()

    def update(self):
        self.basis = []

        x = np.append(self.c[0],self.c[0,0:ORDER])
        y = np.append(self.c[1],self.c[1,0:ORDER])
        self.c_periodic = np.array([x,y])
        self.n_c_periodic = len(x)
        
        for i in range(self.n_c_periodic):
            self.basis.append(BSpline.basis_element(self.t[i:i+ORDER+2],False))
        
        self.bsplx = BSpline(self.t,x,ORDER,False)
        self.bsply = BSpline(self.t,y,ORDER,False)
        self.derx1 = self.bsplx.derivative(1)
        self.dery1 = self.bsply.derivative(1)
        self.derx2 = self.bsplx.derivative(2)
        self.dery2 = self.bsply.derivative(2)

        # Sample some values
        self.sample_nb = NB_POINTS_BSPL
        self.sample_t = np.linspace(self.t_min,self.t_max,self.sample_nb,endpoint=True)
        self.sample_values = np.zeros((self.sample_nb,2))
        for i in range(self.sample_nb):
            self.sample_values[i] = self.estimate(self.sample_t[i])

        #self.plot_curve(True)
        #plt.show()

    def estimate_with_basis(self,tk):
        tk = self.modulo_tk(tk)  
        b = self.get_basis(tk)
        pt = np.array([0.0,0.0])
        for i in range(self.n_c_periodic):
            pt[0] += b[i]*self.c_periodic[0][i]
            pt[1] += b[i]*self.c_periodic[1][i]
        return pt

    def get_basis(self,tk):
        tk = self.modulo_tk(tk)
        b_term = np.zeros(self.n_c_periodic)
        for i in range(self.n_c_periodic):
            tmp = self.basis[i](tk)
            if not np.isnan(tmp):
                b_term[i] = tmp
        return b_term

    def modulo_tk(self,tk):
        return (tk-self.t_min)%(self.t_max-self.t_min) + self.t_min

    def estimate(self,tk):
        tk = self.modulo_tk(tk)        
        return np.array([self.bsplx(tk),self.bsply(tk)])

    def derivative1(self,tk):
        tk = self.modulo_tk(tk)
        return np.array([self.derx1(tk),self.dery1(tk)])

    def derivative2(self,tk):
        tk = self.modulo_tk(tk)
        return np.array([self.derx2(tk),self.dery2(tk)])

    def plot_curve(self,plot_ctrl_pts=False):
        plt.figure(utility.figMerge)
        u = np.linspace(self.t_min,self.t_max,self.sample_nb)
        pt = np.zeros((self.sample_nb,2))
        #pt2 = np.zeros((self.sample_nb,2))
        for i in range(self.sample_nb):
            pt[i] = self.estimate(u[i])
            #pt2[i] = self.estimate_with_basis(u[i])
        plt.plot(pt[:,0],pt[:,1],'b',linewidth=2)
        #plt.plot(pt2[:,0],pt2[:,1],'.r')
        if plot_ctrl_pts:
            plt.plot(np.append(self.c[0,:],self.c[0,0]),np.append(self.c[1,:],self.c[1,0]),'g',linewidth=2)
            

def init_SDM(points):
    origin,radius = utility.fit_circle(points)

    n = NB_OF_CTRL_POINTS
    n_sect = np.zeros(n,'int')
    d_sect = [np.array([])]*n
    median_sect = np.zeros(n)
    amin = np.zeros(n)
    amid = np.zeros(n)
    amax = np.zeros(n)
    for i in range(n):
        amin[i] = 2*math.pi*i/n 
        amid[i] = 2*math.pi*(i+0.5)/n 
        amax[i] = 2*math.pi*(i+1)/n
    for i in range(len(points)):
        angle = np.arctan2(points[i][1]-origin[1],points[i][0]-origin[0]) % (2*math.pi)
        for j in range(n):
            if amin[j] <= angle and angle < amax[j]:
                n_sect[j] += 1
                d_sect[j] = np.append(d_sect[j],utility.euclidian_dist(points[i],origin))

    for i in range(n):
        if n_sect[i] > 0:
            median_sect[i] = np.median(d_sect[i])

    for i in range(n):
        if n_sect[i] == 0:
            j = (i-1)%n
            while n_sect[j] == 0:
                j = (j-1)%n
            down = j
            j = (i+1)%n
            while n_sect[j] == 0:
                j = (j+1)%n
            up = j
            median_sect[i] = (median_sect[down] + median_sect[up]) / 2.0

    P = np.zeros((n,2))
    for i in range(n):
        P[i] = [origin[0]+(np.cos(amid[i])*median_sect[i]),origin[1]+(np.sin(amid[i])*median_sect[i])]

    # Circle plotting
    #plt.figure(utility.figMerge)
    #a = np.linspace(0,2*np.pi,300)
    #plt.plot(origin[0]+radius*np.cos(a), origin[1]+radius*np.sin(a), '-r',linewidth=2)
    #plt.plot(P[:,0], P[:,1],'-m',linewidth=2)

    Px_per = np.append(P[:,0],P[0,0])
    Py_per = np.append(P[:,1],P[0,1])
    tck,u = splprep([Px_per, Py_per],s=0,k=ORDER,per=True)
    C = tck[1]
    C = np.transpose(C)
    C = C[:-ORDER]
    #plt.plot(C[:,0], C[:,1],'-c',linewidth=2)
    
    return C

def find_tk_foot_point(bspl,point):
    closest_index = distance.cdist([point], bspl.sample_values).argmin()
    tk = bspl.sample_t[closest_index]
    return tk

    """
    iter_max = 100
    iter_nb = 0
    epsi_deri = 1*10**(-6)
    epsi = 0.001
    a = bspl.sample_t[closest_index-1]
    b = bspl.sample_t[(closest_index+1)%bspl.sample_nb]

    def fun(x):
        pt1_tk = bspl.estimate(x)
        pt2_tk = bspl.estimate(x+epsi_deri)
        d1 = utility.euclidian_dist(pt1_tk,point)
        d2 = utility.euclidian_dist(pt2_tk,point)
        y = (d2-d1)/epsi_deri
        return y
        
    # Dichotomy
    while iter_nb < iter_max or b-a > epsi:
        iter_nb += 1
        m = (a+b)/2
        if fun(a)*fun(m) < 0:
            b = m
        else:
            a = m
    return bspl.modulo_tk(tk)
    """

def squared_dist(dist,rad,Ta_tk,No_tk,tk,neigh,bspl,points):
    esd = np.zeros((2*bspl.n_c,2*bspl.n_c))
    const = np.zeros(2*bspl.n_c)
    n = bspl.n_c

    bi_bj = np.zeros((n,n))
    for k in range(len(points)):
        # Basis elements
        tmp_b_terms = bspl.get_basis(tk[k])
        tmp_b_terms[:ORDER] += tmp_b_terms[-ORDER:]
        b_terms = tmp_b_terms[:-ORDER]

        # SDM
        if dist[k] >= 0 and dist[k] < rad[k]:            
            neigh_pt_norm = (neigh[k][0]-points[k][0])*No_tk[k,0] + (neigh[k][1]-points[k][1])*No_tk[k,1]
            neigh_pt_norm_Nox = neigh_pt_norm * No_tk[k,0]
            neigh_pt_norm_Noy = neigh_pt_norm * No_tk[k,1]
            Nox_Nox = No_tk[k,0]**2
            Nox_Noy = No_tk[k,0]*No_tk[k,1]
            Noy_Noy = No_tk[k,1]**2

            for i in range(n):
                for j in range(n):
                    bi_bj = b_terms[i]*b_terms[j]
                    esd[i,j]     += bi_bj*Nox_Nox
                    esd[i,j+n]   += bi_bj*Nox_Noy
                    esd[i+n,j]   += bi_bj*Nox_Noy 
                    esd[i+n,j+n] += bi_bj*Noy_Noy
                const[i]   -= b_terms[i]*neigh_pt_norm_Nox
                const[i+n] -= b_terms[i]*neigh_pt_norm_Noy

        elif dist[k] < 0:
            neigh_pt_norm = (neigh[k][0]-points[k][0])*No_tk[k,0] + (neigh[k][1]-points[k][1])*No_tk[k,1]
            neigh_pt_norm_Nox = neigh_pt_norm * No_tk[k,0]
            neigh_pt_norm_Noy = neigh_pt_norm * No_tk[k,1]
            prodNeighPtTang = (neigh[k][0]-points[k][0])*Ta_tk[k,0] + (neigh[k][1]-points[k][1])*Ta_tk[k,1]
            dist_distRad = dist[k]/(dist[k]-rad[k])
            dist_distRad_Tax_Tax = dist_distRad * (Ta_tk[k,0]**2)
            dist_distRad_Tax_Tay = dist_distRad * Ta_tk[k,0]*Ta_tk[k,1]
            dist_distRad_Tay_Tay = dist_distRad * (Ta_tk[k,1]**2)
            Nox_Nox = No_tk[k,0]**2
            Nox_Noy = No_tk[k,0]*No_tk[k,1]
            Noy_Noy = No_tk[k,1]**2
            
            for i in range(n):
                for j in range(n):
                    bi_bj = b_terms[i]*b_terms[j] 
                    esd[i][j]    += bi_bj*dist_distRad_Tax_Tax + bi_bj*Nox_Nox
                    esd[i,j+n]   += bi_bj*dist_distRad_Tax_Tay + bi_bj*Nox_Noy
                    esd[i+n,j]   += bi_bj*dist_distRad_Tax_Tay + bi_bj*Nox_Noy
                    esd[i+n,j+n] += bi_bj*dist_distRad_Tay_Tay + bi_bj*Noy_Noy 
                const[i]   -= dist_distRad*b_terms[i]*Ta_tk[k,0]*prodNeighPtTang + b_terms[i]*neigh_pt_norm_Nox
                const[i+n] -= dist_distRad*b_terms[i]*Ta_tk[k,1]*prodNeighPtTang + b_terms[i]*neigh_pt_norm_Noy
        
        else:
            cust_print("Error rad < dist: " + str(rad) + '<',dist)
            raise

    return esd,const


def mark_zeros_line(bspl,esd,tresh):
    zeros = np.full(bspl.n_c*2, False)
    for i in range(2*bspl.n_c):
        sum_line = 0
        for j in range(2*bspl.n_c):
            sum_line += esd[i][j]
        if np.abs(sum_line) < tresh:
            zeros[i] = True
    
    cpt = 0
    for i in zeros:
        if i == False:
            cpt+=1
    if cpt < 2:
        cust_print("Cannot minimize SD error!")
        raise
    return zeros


def apply_zeros_constraints(bspl,esd,const,zeros):
    for i in range(bspl.n_c*2):
        if zeros[i]:
            for j in range(bspl.n_c*2):
                esd[i][j] = 0.0
                esd[j][i] = 0.0

    for i in range(bspl.n_c*2):
        if zeros[i]:
            esd[i][i] = 1.0
            const[i] = 0.0
    
    return esd,const


def move_ctrl_points(bspl,P,D,zeros):
    for i in range(bspl.n_c):
        if not zeros[i]:
            P[0][i] += D[i]
        if not zeros[bspl.n_c+i]:
            P[1][i] += D[bspl.n_c+i]

    for i in range(bspl.n_c):
        if zeros[i]:
            j = (i-1)%bspl.n_c
            diff1 = 1
            while zeros[j] == True:
                j = (j-1)%bspl.n_c
                diff1 += 1 
            x1 = P[0][j]
            j = (i+1)%bspl.n_c
            diff2 = 1
            while zeros[j] == True:
                j = (j+1)%bspl.n_c
                diff2 += 1 
            x2 = P[0][j]
            x_mid = x1 + (x2-x1)/(diff1+diff2)
            x = x_mid + (P[0][i]-x_mid)/2
            P[0][i] = x
    for i in range(bspl.n_c):
        if zeros[i+bspl.n_c]:
            j = (i-1)%(bspl.n_c) 
            diff1 = 1
            while zeros[j+bspl.n_c] == True:
                j = (j-1)%bspl.n_c
                diff1 += 1 
            y1 = P[1][j]
            j = (i+1)%bspl.n_c
            diff2 = 1
            while zeros[j+bspl.n_c] == True:
                j = (j+1)%bspl.n_c
                diff2 += 1 
            y2 = P[1][j]
            y_mid = y1 + (y2-y1)/(diff1+diff2)
            y = y_mid + (P[1][i]-y_mid)/2
            P[1][i] = y

    return P    

def compute_approx_error(dist):
    n = len(dist)
    return np.sqrt((1/n)*np.sum(dist**2))

def compute_points_attributes(points,bspl):
    tk = np.zeros(len(points))
    neigh = np.zeros((len(points),2))
    No_tk = np.zeros((len(points),2))
    Ta_tk = np.zeros((len(points),2))
    rad = np.zeros(len(points))
    dist = np.zeros(len(points))
    for i in range(len(points)):
        # Find foot points
        tk[i] = find_tk_foot_point(bspl,points[i])
        # Foot point
        neigh[i] = bspl.estimate(tk[i])
        # Derivatives
        der_1 = bspl.derivative1(tk[i])
        der_2 = bspl.derivative2(tk[i])
        # Unit tangeant vector and unit normal vector at the point neigh
        Ta_tk[i] = np.array([der_1[0],der_1[1]])
        Ta_tk[i] = Ta_tk[i]/utility.euclidian_dist([0.0,0.0],Ta_tk[i])
        No_tk[i] = np.array([der_2[0],der_2[1]])
        No_tk[i] = No_tk[i]/utility.euclidian_dist([0.0,0.0],No_tk[i])
        # Radius
        rad[i] = abs(((der_1[0]**2 + der_1[1]**2)**(3/2))/(der_1[0]*der_2[1]-der_2[0]*der_1[1]))
        linear_comb = (points[i]-neigh[i])*No_tk[i]
        # Distance
        dist[i] = utility.euclidian_dist(neigh[i],points[i])
        if linear_comb[np.argmax(np.abs(linear_comb))] < 0:
            dist[i] *= -1
    return dist,rad,Ta_tk,No_tk,tk,neigh

def compute_regularization(bspl):
    n = bspl.n_c
    const = np.zeros(2*n)
    reg = np.zeros((2*n,2*n))

    for i in range(bspl.n_c):
        i_prev = int((i+1)%n)
        i_next = int((i-1)%n)
        mid_x = (bspl.c[0][i_prev] + bspl.c[0][i_next]) / 2
        mid_y = (bspl.c[1][i_prev] + bspl.c[1][i_next]) / 2

        dx = mid_x - bspl.c[0][i]
        dy = mid_y - bspl.c[1][i] 

        reg[i][i] = 1
        reg[i+n][i+n] = 1
        const[i] = dx
        const[i+n] = dy
    return reg,const

"""
    n = bspl.n_c
    const = np.zeros(2*n)
    reg = np.zeros((2*n,2*n))
    return reg,const
"""

# Return an array of indices of the non outlier points
def get_non_outliers(dist):
    non_outliers = np.array([],'int')
    six_sig = 6 * np.std(np.abs(dist))
    for i,d in enumerate(dist):
        if np.abs(d) <= six_sig:
            non_outliers = np.append(non_outliers, i)
    return non_outliers  

    
def iter_SDM(points,bspl):
    iter_max = ITER_MAX
    nb_iter = 0
    temp_approx_error = math.inf
    
    while True:  
        # Stop condition
        if nb_iter >= iter_max:
            break

        # Compute point attributes
        dist,rad,Ta_tk,No_tk,tk,neigh = compute_points_attributes(points,bspl)
        non_out_ind = get_non_outliers(dist)

        # Remove the outliers
        points = points[non_out_ind]
        dist = dist[non_out_ind]
        rad = rad[non_out_ind]
        Ta_tk = Ta_tk[non_out_ind]
        No_tk = No_tk[non_out_ind]
        tk = tk[non_out_ind]
        neigh = neigh[non_out_ind]

        # Approximation error
        approx_error = compute_approx_error(dist)
        if nb_iter == 0:
            cust_print("    Fit average error init: " + str(approx_error))
        else:
            cust_print("    Fit average error: " + str(approx_error))

        # Stop conditions
        if approx_error <= APPROXIMATION_ERROR_THRESHOLD:
            break
        if approx_error >= temp_approx_error:
            bspl = copy.copy(temp_bspl)
            break 

        # Temp variables
        temp_approx_error = approx_error
        temp_bspl = copy.copy(bspl)

        # Objective function minimization
        esd,esd_const = squared_dist(dist,rad,Ta_tk,No_tk,tk,neigh,bspl,points)
        reg,reg_const = compute_regularization(bspl)

        # Regularization
        fsd = 0.5*esd + REGUL_WEIGHT*reg
        const = 0.5*esd_const + REGUL_WEIGHT*reg_const

        # Zeros constraints for robustness
        zeros = mark_zeros_line(bspl,fsd,ZERO_LINE_THRESHOLD)
        fsd,const = apply_zeros_constraints(bspl,fsd,const,zeros)

        # System solving
        D = solve(fsd,const)

        # Update spline
        #print(np.mean(np.abs(D)))
        P = move_ctrl_points(bspl,bspl.c,1*D,zeros)
        bspl.update()
        #bspl.plot_curve(True)
        #plt.show()

        nb_iter += 1  
    return bspl, approx_error


def SDM_algorithm(points):
    P = init_SDM(points)
    
    # Evaluate bspline
    bspl = bspline(P)
    #bspl.plot_curve(True)
    #plt.show()
    
    try:
        bspl, error = iter_SDM(points,bspl)
    except:
        cust_print("An error occured while reconstructing the contour!")
    bspl.plot_curve(False)
    return bspl, error

def compute_circumference(bspl):
    n = 1000
    t = np.linspace(bspl.t_min,bspl.t_max,n,endpoint=True)
    circ = 0.0
    for i in range(n-1):
        circ += utility.euclidian_dist(bspl.estimate(t[i]),bspl.estimate(t[i+1]))
    return circ

def contour():
    #path = r'C:\Users\Toshiba\Documents\Vincent MAIRE\lidar_waist_scan\data\data_test.txt'
    #datas = np.loadtxt(path, dtype='d', delimiter=' ')
    #plt.figure(utility.figMerge)
    #plt.gca().set_aspect('equal')
    #plt.plot(datas[:,0], datas[:,1], '.k', ms=3)
    #for i in range(len(datas)):
    #    points[i] = [datas[i,0],datas[i,1]]
    
    datas = utility.mergedPointsXY
    points = np.zeros((len(datas),2))
    for i in range(len(datas)):
        points[i] = [datas[i].x,datas[i].y]

    bspl, error = SDM_algorithm(points)
    circum = compute_circumference(bspl)

    cust_print('\nCircumference: ' + str(format(circum, '.2f')) + 'mm')

    
