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

NB_OF_CTRL_POINTS_START = 6
ORDER = 3
NB_POINTS_BSPL = 100
APPROXIMATION_ERROR_TRESHOLD = 2
APPROXIMATION_CONV_TRESHOLD = 0.5
ITER_MAX = 1
REGUL_WEIGHT = 0.0001


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

    def add_ctrl_point(self,pos,points,tk,dist):
        pos = int(pos)
        c = self.c
        #new_ctrl_x = self.estimate((self.t[pos]+self.t[pos+1])/2.0)[0]
        #new_ctrl_y = self.estimate((self.t[pos]+self.t[pos+1])/2.0)[1]
        new_ctrl_x = (c[0][pos]+c[0][(pos+1)%self.n_c])/2.0
        new_ctrl_y = (c[1][pos]+c[1][(pos+1)%self.n_c])/2.0
        c = np.insert(c,pos+1,[new_ctrl_x,new_ctrl_y],axis=1)
        self.c = c

        t = self.t
        new_t = (self.t[pos] + self.t[pos+1]) / 2.0
        t = np.insert(t,pos+1,new_t)
        self.t = t
    

    def plot_curve(self,plot_ctrl_pts=False):
        u = np.linspace(self.t_min,self.t_max,self.sample_nb)
        pt = np.zeros((self.sample_nb,2))
        pt2 = np.zeros((self.sample_nb,2))
        for i in range(self.sample_nb):
            pt[i] = self.estimate(u[i])
            pt2[i] = self.estimate_with_basis(u[i])
        plt.plot(pt[:,0],pt[:,1],'k')
        #plt.plot(pt2[:,0],pt2[:,1],'.r')
        if plot_ctrl_pts:
            plt.plot(self.c[0],self.c[1],'k')
            

def init_SDM(points):
    max_x = np.max(points[:,0])
    min_x = np.min(points[:,0])
    max_y = np.max(points[:,1])
    min_y = np.min(points[:,1])
    #radius = np.max([max_x-min_x,max_y-min_y])/2 + 20
    #center= np.array([min_x+radius-20,min_y+radius-20])
    radius = 150 #constants.lidarsDist
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
        
    #plt.plot([neigh[0],neigh[0]+dist*No_tk[0]],[neigh[1],neigh[1]+dist*No_tk[1]],'k')
    # SDM
    prodNeighPtNorm = (neigh-point)[0]*No_tk[0] + (neigh-point)[1]*No_tk[1]
    if dist >= 0 and dist < rad:
        for i in range(bspl.n_c):
            for j in range(bspl.n_c):
                temp = b_terms[i]*b_terms[j]
                esd[i+0][j+0]               += temp*(No_tk[0]**2)    
                esd[i+0][j+bspl.n_c]        += temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+0]        += temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+bspl.n_c] += temp*(No_tk[1]**2)
            const[i+0]        -= b_terms[i]*No_tk[0]*prodNeighPtNorm
            const[i+bspl.n_c] -= b_terms[i]*No_tk[1]*prodNeighPtNorm
    elif dist < 0:
        prodNeighPtTang = (neigh-point)[0]*Ta_tk[0] + (neigh-point)[1]*Ta_tk[1]
        for i in range(bspl.n_c):
            for j in range(bspl.n_c):
                temp = b_terms[i]*b_terms[j] 
                esd[i+0][j+0]               += temp*(dist/(dist-rad))*(Ta_tk[0]**2) + temp*(No_tk[0]**2)
                esd[i+0][j+bspl.n_c]        += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+0]        += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+bspl.n_c][j+bspl.n_c] += temp*(dist/(dist-rad))*(Ta_tk[1]**2) + temp*(No_tk[1]**2)
            const[i+0]        -= ((dist/(dist-rad))*b_terms[i]*Ta_tk[0]*prodNeighPtTang + b_terms[i]*No_tk[0]*prodNeighPtNorm)
            const[i+bspl.n_c] -= ((dist/(dist-rad))*b_terms[i]*Ta_tk[1]*prodNeighPtTang + b_terms[i]*No_tk[1]*prodNeighPtNorm)
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
    """
    n = bspl.n_c
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

    nb_values_integ = 100 
    integ_x = np.linspace(t[0],t[-1],nb_values_integ)
    spacing = integ_x[1] - integ_x[0]
    for i in range(n):
        for j in range(n):
            for m in integ_x:
                b = bspl.get_basis(m)
                I[i][j] += ( a[(i-2)%n]*b[(i-2)%n]*b[j]
                             -(a[(i-1)%n]+c[(i-1)%n])*b[(i-1)%n]*b[j]
                             +c[i]*b[i]*b[j] ) * spacing
    """
    n = bspl.n_c
    const = np.zeros(2*n)
    reg = np.zeros((2*n,2*n))
    """    
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
    """
    return reg,const
    
    
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
            esd,esd_const = squared_dist(esd,const,dist[i],rad[i],Ta_tk[i],No_tk[i],tk[i],neigh[i],bspl,points[i])
        reg,reg_const = compute_regularization(bspl)

        fsd = 0.5*esd + REGUL_WEIGHT*reg
        const = 0.5*esd_const + REGUL_WEIGHT*reg_const

        # Zeros constraints for robustness
        zeros = mark_zeros_line(bspl,fsd,0.5)
        fsd,const = apply_zeros_constraints(bspl,fsd,const,zeros)

        # System solving
        D = lstsq(fsd,const)[0]

        # Update spline
        P = move_ctrl_points(bspl,bspl.c,D,zeros)
        bspl.update()

        # Approximation error
        dist,rad,Ta_tk,No_tk,tk,neigh = compute_points_attributes(points,bspl)   
        Ej = compute_approx_error(points,bspl,tk,dist)
        approx_error = (1/bspl.n_c)*np.sum(Ej)
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
        #if np.abs(temp_error[1]-temp_error[0]) <= epsi_convergence:
        #temp_error = [math.inf,math.inf]
        # Add ctrl point
        #print("New ctrl point added")
        #bspl.plot_curve(True)
        #P = bspl.add_ctrl_point(np.argmax(Ej),points,tk,dist)
        #bspl.update()
        #dist,rad,Ta_tk,No_tk,tk,neigh = compute_points_attributes(points,bspl)
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

    try:
        bspl = iter_SDM(points,bspl)
    except:
        print("An error occured")
    bspl.plot_curve(True)

def contour():
    datas = utility.mergedPointsXY
    points = np.zeros((len(datas),2))

    #radius = constants.lidarsDist*0.9
    #center = [0.0,0.0]
    #n = NB_POINTS_BSPL
    #points = np.zeros((n,2))
    #for i in range(n):
    #    points[i] = np.array([radius*np.cos(i*2.0*math.pi/n)+center[0],radius*np.sin(i*2.0*math.pi/n)+center[1]])
    #plt.plot(points[:,0],points[:,1],'xg')

    
    for i in range(len(datas)):
        points[i] = [datas[i].x,datas[i].y]
    
    SDM_algorithm(points)
