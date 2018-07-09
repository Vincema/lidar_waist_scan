from bb_serial import cust_print, cust_read
import utility
import constants
import math
import numpy as np
import matplotlib.pyplot as plt
import copy
from scipy.interpolate import BSpline, splev, splprep, splrep, CubicSpline
from scipy.linalg import solve,lstsq
from scipy.spatial import distance
import time

NB_OF_CTRL_POINTS = 10
ORDER = 3
NB_POINTS_BSPL = 50*NB_OF_CTRL_POINTS
APPROXIMATION_ERROR_TRESHOLD = 1.0
ITER_MAX = 10
REGUL_WEIGHT = 0.001


class bspline:
    def __init__(self,ctrl_pts):
        x = ctrl_pts[:,0]
        y = ctrl_pts[:,1]
        self.n_c = len(x)
        self.t = np.linspace(0,len(x)+ORDER+1-1,len(x)+ORDER+1)
        self.t_max = self.n_c
        self.t_min = 0.0
        self.c = np.array([x,y])
        self.update()

    def update(self):
        self.n_c = len(self.c[0])
        self.basis = []
        for i in range(self.n_c):
            self.basis.append(BSpline.basis_element(self.t[i:i+ORDER+2],False))
        # Rearrange the basis element because the i-th basis element defines the curve at the i-th+ORDER ctrl point
        for i in range(self.n_c-ORDER+1):
            self.basis.append(self.basis.pop(0))

        n = self.n_c
        x = np.zeros(n+1)
        y = np.zeros(n+1)
        t = self.t[:n+1]
        for i in range(n+1):
            [x[i],y[i]] = self.estimate_with_basis(t[i])
        x[-1] = x[0]
        y[-1] = y[0]
        self.bsplx = CubicSpline(t,x,bc_type='periodic')
        self.bsply = CubicSpline(t,y,bc_type='periodic')
        self.derx1 = self.bsplx.derivative(1)
        self.dery1 = self.bsply.derivative(1)
        self.derx2 = self.bsplx.derivative(2)
        self.dery2 = self.bsply.derivative(2)

        # Sample some values
        self.sample_nb = NB_POINTS_BSPL
        self.sample_t = np.linspace(self.t_min,self.t_max,self.sample_nb)
        self.sample_values = np.zeros((self.sample_nb,2))
        for i in range(self.sample_nb):
            self.sample_values[i] = self.estimate(self.sample_t[i])

    def get_basis(self,tk):
        tk = self.modulo_tk(tk)
        b_term = np.zeros(self.n_c)
        for i in range(self.n_c):
            if not np.isnan(self.basis[i](tk)):
                b_term[i] += self.basis[i](tk)
            if not np.isnan(self.basis[i](tk+self.t_max)):
                b_term[i] += self.basis[i](tk+self.t_max)
        return b_term

    def modulo_tk(self,tk):
        return (tk-self.t_min)%(self.t_max-self.t_min) + self.t_min

    def estimate(self,tk):
        tk = self.modulo_tk(tk)        
        return np.array([self.bsplx(tk),self.bsply(tk)])

    def estimate_with_basis(self,tk):
        tk = self.modulo_tk(tk)  
        b = self.get_basis(tk)
        pt = np.array([0.0,0.0])
        for i in range(self.n_c):
            pt[0] += b[i]*self.c[0][i]
            pt[1] += b[i]*self.c[1][i]
        return pt

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
        plt.plot(pt[:,0],pt[:,1],ms=3.5)
        #plt.plot(pt2[:,0],pt2[:,1],'.r')
        if plot_ctrl_pts:
            plt.plot(self.c[0],self.c[1],'g')
            

def init_SDM(points):
    origin,radius = utility.fit_circle(points)

    n = NB_OF_CTRL_POINTS
    n_sect = np.zeros(n,'int')
    mean_sect = np.zeros(n)
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
                mean_sect[j] += utility.euclidian_dist(points[i],origin)

    for i in range(n):
        if n_sect[i] > 0:
            mean_sect[i] /= n_sect[i]

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
            mean_sect[i] = (mean_sect[down] + mean_sect[up]) / 2.0

    P = np.zeros((n,2))
    for i in range(n):
        P[i] = [origin[0]+(np.cos(amid[i])*mean_sect[i]),origin[1]+(np.sin(amid[i])*mean_sect[i])]
    return P

def find_tk_foot_point(bspl,point):
    closest_index = distance.cdist([point], bspl.sample_values).argmin()
    tk = bspl.sample_t[closest_index]
    return tk
   
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

def squared_dist(dist,rad,Ta_tk,No_tk,tk,neigh,bspl,points):
    esd = np.zeros((2*bspl.n_c,2*bspl.n_c))
    const = np.zeros(2*bspl.n_c)
    n = bspl.n_c

    bi_bj = np.zeros((n,n))
    for k in range(len(points)):
        # Basis elements
        b_terms = bspl.get_basis(tk[k])

        # SDM
        if dist[k] >= 0 and dist[k] < rad[k]:
            neigh_pt_norm = -dist[k]*No_tk[k,0]**2 + -dist[k]*No_tk[k,1]**2 #(neigh[0]-point[0])*No_tk[0] + (neigh[1]-point[1])*No_tk[1]
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
            neigh_pt_norm = -dist[k]*No_tk[k,0]**2 + -dist[k]*No_tk[k,1]**2 #(neigh[0]-point[0])*No_tk[0] + (neigh[1]-point[1])*No_tk[1]
            neigh_pt_norm_Nox = neigh_pt_norm * No_tk[k,0]
            neigh_pt_norm_Noy = neigh_pt_norm * No_tk[k,1]
            # prodNeighPtTang = 0.0 #(neigh[0]-point[0])*Ta_tk[0] + (neigh[1]-point[1])*Ta_tk[1]
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
                const[i]   -= b_terms[i]*neigh_pt_norm_Nox  #dist_distRad*b_terms[i]*Ta_tk[0]*prodNeighPtTang 
                const[i+n] -= b_terms[i]*neigh_pt_norm_Noy  #dist_distRad*b_terms[i]*Ta_tk[1]*prodNeighPtTang
        else:
            cust_print("Error rad < dist: " + str(rad) + '<',dist)
            raise

    return esd,const


def mark_zeros_line(bspl,esd,tresh):
    zeros = []
    for i in range(bspl.n_c*2):
        all_zeros = True
        for j in range(bspl.n_c*2):
            if np.abs(esd[i][j]) >= tresh:
                all_zeros = False
        zeros.append(all_zeros)
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
    return reg,const

"""
    n = bspl.n_c
    const = np.zeros(2*n)
    reg = np.zeros((2*n,2*n))
    k = ORDER
    t = bspl.t
    P_x = bspl.c[0]
    P_y = bspl.c[1]

    a = np.zeros(n)
    c = np.zeros(n)
    I = np.zeros((n,n))
    for i in range(n):
        a[i] = 1.0 / ((t[i+k]-t[i+2])*(t[i+k+1]-t[i+2]))
        c[i] = 1.0 / ((t[i+k]-t[i+2])*(t[i+k]-t[i+1]))

    nb_values_integ = 20 
    integ_x = np.linspace(t[0],t[-1],nb_values_integ)
    spacing = integ_x[1] - integ_x[0]
    for i in range(n):
        for j in range(n):
            prev = 0.0
            for m in integ_x:
                b = bspl.get_basis(m)
                temp = ( a[(i-2)%n]*b[(i-2)%n]*b[j]
                         -(a[(i-1)%n]+c[(i-1)%n])*b[(i-1)%n]*b[j]
                         +c[i]*b[i]*b[j] )
                I[i][j] += (temp + prev) * spacing / 2.0
                prev = temp
  
    temp1 = 2*((k-1)**2)*((k-2)**2)
    for i in range(n):
        temp2_x = 0
        for j in range(n-3):
            temp2_x += (a[j]*P_x[j+2]-(a[j]+c[j])*P_x[j+1]+c[j]*P_x[j])*I[i][j]
        temp2_y = 0
        for j in range(n-3):
            temp2_y += (a[j]*P_y[j+2]-(a[j]+c[j])*P_y[j+1]+c[j]*P_y[j])*I[i][j]
        const[i] -= temp1*temp2_x
        const[i+n] -= temp1*temp2_y

        temp3 = a[(j-2)%n]*I[i][(j-2)%n]-(a[(j-1)%n]+c[(j-1)%n])*I[i][(j-1)%n]+c[j]*I[i][j]
        for j in range(n):
            reg[i+0][j+0] = temp1*temp3
            reg[i+0][j+n] = 0
            reg[i+n][j+0] = 0
            reg[i+n][j+n] = temp1*temp3
    return reg,const
"""

    
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

        # Approximation error
        approx_error = compute_approx_error(dist)
        if nb_iter == 0:
            cust_print("    Fit average error init: " + str(approx_error))
        else:
            cust_print("    Fit average error: " + str(approx_error))

        # Stop conditions
        if approx_error <= APPROXIMATION_ERROR_TRESHOLD:
            break
        if approx_error >= temp_approx_error:
            bspl = temp_bspl
            break 

        # Temp variables
        temp_approx_error = approx_error
        temp_bspl = copy.copy(bspl)

        # Objective function minimization
        esd,esd_const = squared_dist(dist,rad,Ta_tk,No_tk,tk,neigh,bspl,points)
        reg,reg_const = compute_regularization(bspl)
    
        fsd = 0.5*esd + REGUL_WEIGHT*reg
        const = 0.5*esd_const + REGUL_WEIGHT*reg_const

        # Zeros constraints for robustness
        zeros = mark_zeros_line(bspl,fsd,0.5)
        fsd,const = apply_zeros_constraints(bspl,fsd,const,zeros)

        # System solving
        D = solve(fsd,const)

        # Update spline
        P = move_ctrl_points(bspl,bspl.c,0.5*D,zeros)
        bspl.update()
        #bspl.plot_curve()
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
        cust_print("An error occured")
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
    datas = utility.mergedPointsXY
    
    #path = constants.dataPath + r'/2dPointsTest.txt'
    #datas = np.loadtxt(path, dtype='d', delimiter=' ')

    #plt.plot(datas[:,0], datas[:,1], '.r', ms=3)
    
    points = np.zeros((len(datas),2))
    for i in range(len(datas)):
        points[i] = [datas[i].x,datas[i].y]
        #points[i] = [datas[i,0],datas[i,1]]

    bspl, error = SDM_algorithm(points)
    circum = compute_circumference(bspl)

    cust_print('\nCircumference: ' + str(format(circum, '.2f')) + 'mm')

    
