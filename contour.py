import utility
import constants
import math
import numpy as np
import matplotlib.pyplot as plt
import skfmm
from scipy.interpolate import BSpline, splev, splprep, splrep, CubicSpline
from scipy.linalg import solve,lstsq
from scipy.spatial import distance
import time

NB_OF_CTRL_POINTS_START = 4
ORDER = 3
GRID_STEP = 0.5  # mm
NB_POINTS_BSPL = 100
APPROXIMATION_ERROR_TRESHOLD = 1
APPROXIMATION_CONV_TRESHOLD = 0.2
ITER_MAX = 20


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

        n = NB_POINTS_BSPL
        x = np.zeros(n)
        y = np.zeros(n)
        t = np.linspace(0,self.t_max,n)
        for i in range(n):
            [x[i],y[i]] = self.estimate_with_basis(t[i])
        x = np.append(x,x[0])
        y = np.append(y,y[0])
        t = np.linspace(0,self.t_max,n+1,endpoint=True)
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

    def add_ctrl_point(self,pos,points,tk,dist):
        pos = int(pos)
        c = self.c
        new_ctrl_x = self.estimate((self.t[pos]+self.t[pos+1])/2.0)[0]
        new_ctrl_y = self.estimate((self.t[pos]+self.t[pos+1])/2.0)[1]
        c = np.insert(c,pos+1,[new_ctrl_x,new_ctrl_y],axis=1)
        self.c = c

        t = self.t
        new_t = (self.t[pos] + self.t[pos+1]) / 2.0
        t = np.insert(t,pos+1,new_t)
        self.t = t
        

        """
        # Uses PERM method (http://mi.eng.cam.ac.uk/~cipolla/publications/article/1999-PAMI-bspline-fitting.pdf)
        sorted_ind = np.argsort(tk)
        sorted_tk = tk[sorted_ind]
        sorted_dist = dist[sorted_ind]

        sub_tk = np.array([])
        sub_dist = np.array([])
        sub_size = 0
        for i in range(len(points)):
            if np.floor(sorted_tk[i]) == pos:
                sub_size += 1
                sub_tk = np.append(sub_tk,sorted_tk[i])
                sub_dist = np.append(sub_dist,sorted_dist[i])

        left_cnt = 0
        right_cnt = sub_size-1
        El = np.zeros(sub_size)
        Er = np.zeros(sub_size)
        alpha_left = sub_dist[0]
        alpha_right = sub_dist[-1]
        for i in range(sub_size):
            beta = sub_dist[i]          
            for j in range(left_cnt):
                El[i] += (alpha_left + j*((beta-alpha_left)/left_cnt) - sub_dist[j])**2
            for j in range(right_cnt):
                Er[i] += (alpha_right + j*((beta-alpha_right)/right_cnt) - sub_dist[j])**2
            left_cnt += 1
            right_cnt -= 1

        print(sub_tk)
        print(El)
        print()
        print(Er)
        print()

        grad_El = np.gradient(El)
        grad_Er = np.gradient(Er)
        hess_El = utility.hessian(El)
        hess_Er = utility.hessian(Er)

        print(hess_El)
        energy_l = 0.5 * np.matmul(np.matmul(grad_El,np.linalg.inv(hess_El)),np.transpose(grad_El))
        energy_r = 0.5 * np.matmul(np.matmul(grad_Er,np.linalg.inv(hess_Er)),np.transpose(grad_Er))
        tot_energy = energy_l+energy_r
        print(tot_energy)
        tk_max_energy = sub_tk[np.argmax(tot_energy)]
        print(tk_max_energy)
        """    
    

    def plot_curve(self,plot_ctrl_pts=False):
        u = np.linspace(self.t_min,self.t_max,self.sample_nb)
        pt = np.zeros((self.sample_nb,2))
        pt2 = np.zeros((self.sample_nb,2))
        for i in range(self.sample_nb):
            pt[i] = self.estimate(u[i])
            #pt2[i] = self.estimate_with_basis(u[i])
        plt.plot(pt[:,0],pt[:,1])
        plt.plot(pt2[:,0],pt2[:,1],'.r')
        if plot_ctrl_pts:
            plt.plot(self.c[0],self.c[1])

"""
def distance_field(bspl):
    size = 2*constants.lidarsDist
    xmin = ymin = -constants.lidarsDist
    xmax = ymax = constants.lidarsDist
    cell_nb = math.ceil(size/GRID_STEP)
    y,x = np.meshgrid(np.linspace(ymin,ymax,cell_nb), np.linspace(xmin,xmax,cell_nb))
    phi = np.ones_like(x)
    for i in range(NB_POINTS_BSPL):
        est = bspl.sample_values[i]
        index_x = int(math.floor(cell_nb*(est[0]-xmin)/(xmax-xmin)))
        index_y = int(math.floor(cell_nb*(est[1]-ymin)/(ymax-ymin)))
        phi[index_x][index_y] = 0
    dist_field = skfmm.distance(phi,dx=GRID_STEP)
    return dist_field,xmin,ymin,xmax,ymax,cell_nb
"""

def init_SDM(points):
    max_x = np.max(points[:,0])
    min_x = np.min(points[:,0])
    max_y = np.max(points[:,1])
    min_y = np.min(points[:,1])
    #radius = np.max([max_x-min_x,max_y-min_y])/2 + 20
    #center= np.array([min_x+radius-20,min_y+radius-20])
    radius = constants.lidarsDist
    center = [0.0,0.0]
    n = NB_OF_CTRL_POINTS_START
    P = np.zeros((n,2))
    for i in range(n):
        P[i] = [radius*np.cos(i*2*math.pi/n)+center[0],radius*np.sin(i*2*math.pi/n)+center[1]]
        
    return P


def find_tk_foot_point(bspl,point):
    dist = np.zeros(bspl.sample_nb)
    for i in range(bspl.sample_nb):
        dist[i] = np.linalg.norm(bspl.sample_values[i]-point)
    tk = bspl.sample_t[np.argmin(dist)]

    """
    start = time.time() 
    end = time.time()
    print(end - start)
    """
    return tk


def squared_dist(esd,const,dist,rad,Ta_tk,No_tk,tk,neigh,bspl,point):
    # Basis elements
    b_terms = bspl.get_basis(tk)
    
    # SDM
    if dist >= 0:
        for i in range(bspl.n_c):
            for j in range(bspl.n_c):
                temp = b_terms[i]*b_terms[j]
                esd[i+0][j+0]               += temp*(No_tk[0]**2)    
                esd[i+0][j+bspl.n_c]        += temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+0]        += temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+bspl.n_c] += temp*(No_tk[1]**2)
            const[i+0]        -= b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+bspl.n_c] -= b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    elif dist < 0:
        for i in range(bspl.n_c):
            for j in range(bspl.n_c):
                temp = b_terms[i]*b_terms[j] 
                esd[i+0][j+0]               += temp*(dist/(dist-rad))*(Ta_tk[0]**2) + temp*(No_tk[0]**2)
                esd[i+0][j+bspl.n_c]        += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+0]        += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+bspl.n_c] += temp*(dist/(dist-rad))*(Ta_tk[1]**2) + temp*(No_tk[1]**2)
            const[i+0]        -= (dist/(dist-rad))*b_terms[i]*Ta_tk[0]*np.sum((neigh-point)*Ta_tk) + b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+bspl.n_c] -= (dist/(dist-rad))*b_terms[i]*Ta_tk[1]*np.sum((neigh-point)*Ta_tk) + b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    else:
        print("Error rad > dist: ",dist,'>',rad)
        plt.show()
        exit()

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
        print("Cannot minimize SD error!")
        exit(1)
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
            

def compute_approx_error(points,bspl,tk,dist):
    m_i = np.zeros(bspl.n_c)
    d_i = np.zeros(bspl.n_c)
    for i in range(len(points)):
        for j in range(bspl.n_c):
            if tk[i] >= bspl.t[j] and tk[i] < bspl.t[j+1]:
                index = j
        m_i[index] += 1
        d_i[index] += np.abs(dist[i])

    Ej = np.zeros(bspl.n_c)
    for i in range(bspl.n_c):
        if m_i[i] > 0:
            Ej[i] = (1/m_i[i])*d_i[i]
        else:
            Ej[i] = 0
    return Ej


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
        Ta_tk[i] = Ta_tk[i]/utility.euclidian_dist([0,0],Ta_tk[i])
        No_tk[i] = np.array([der_2[0],der_2[1]])
        No_tk[i] = No_tk[i]/utility.euclidian_dist([0,0],No_tk[i])
        # Radius
        rad[i] = abs(((der_1[0]**2 + der_1[1]**2)**(3/2))/(der_1[0]*der_2[1]-der_2[0]*der_1[1]))
        linear_comb = (points[i]-neigh[i])*No_tk[i]
        # Distance
        dist[i] = utility.euclidian_dist(neigh[i],points[i])
        if linear_comb[np.argmax(np.abs(linear_comb))] < 0:
            dist[i] *= -1
    return dist,rad,Ta_tk,No_tk,tk,neigh
            
def iter_SDM(points,bspl):
    iter_max = ITER_MAX
    nb_iter = 0
    temp_error = [math.inf,math.inf]
    
    # Compute points attributes
    dist,rad,Ta_tk,No_tk,tk,neigh = compute_points_attributes(points,bspl)
    
    while True:
        nb_iter += 1    
        
        # Objective function minimization
        esd = np.zeros((2*bspl.n_c,2*bspl.n_c))
        const = np.zeros(2*bspl.n_c)
        for i in range(len(points)):
            esd,const = squared_dist(esd,const,dist[i],rad[i],Ta_tk[i],No_tk[i],tk[i],neigh[i],bspl,points[i])
        esd *= 0.5
        const *= 0.5

        # Zeros constraints for robustness
        zeros = mark_zeros_line(bspl,esd,0.5)
        esd,const = apply_zeros_constraints(bspl,esd,const,zeros)

        # System solving
        D = lstsq(esd,const)[0]

        # Update spline
        P = move_ctrl_points(bspl,bspl.c,D,zeros)
        bspl.update()

        # Approximation error
        dist,rad,Ta_tk,No_tk,tk,neigh = compute_points_attributes(points,bspl)   
        Ej = compute_approx_error(points,bspl,tk,dist)
        #approx_error = (1/bspl.n_c)*np.sum(Ej)
        approx_error = np.max(Ej)
        temp_error[1] = temp_error[0]
        temp_error[0] = approx_error 
        print(approx_error)

        # Stop condition
        epsi_approx_error = APPROXIMATION_ERROR_TRESHOLD
        epsi_convergence = APPROXIMATION_CONV_TRESHOLD
        if approx_error <= epsi_approx_error:
            break
        if nb_iter >= iter_max:
            break
        if np.abs(temp_error[1]-temp_error[0]) <= epsi_convergence:
            temp_error = [math.inf,math.inf]
            # Add ctrl point
            print("New ctrl point added")
            #bspl.plot_curve(True)
            P = bspl.add_ctrl_point(np.argmax(Ej),points,tk,dist)
            bspl.update()
            # Compute points attributes
            dist,rad,Ta_tk,No_tk,tk,neigh = compute_points_attributes(points,bspl)
            #bspl.plot_curve(True)
            #plt.show()

        #bspl.plot_curve(True)
        #plt.show()

    return bspl


def SDM_algorithm(points):
    P = init_SDM(points)
    
    # Evaluate bspline
    bspl = bspline(P)
    bspl.plot_curve(True)

    bspl = iter_SDM(points,bspl)
    bspl.plot_curve(True)

def contour():
    datas = utility.mergedPointsXY
    points = np.zeros((len(datas),2))
    for i in range(len(datas)):
        points[i] = [datas[i].x,datas[i].y]
    
    SDM_algorithm(points)
