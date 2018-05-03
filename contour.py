import utility
import constants
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline, splev, splprep, splrep, CubicSpline
from scipy.linalg import solve,lstsq

NB_OF_CTRL_POINTS = 20
ORDER = 3

class bspline:
    def __init__(self,ctrl_pts):
        x = ctrl_pts[:,0]
        y = ctrl_pts[:,1]
        self.update(np.array([x,y]))

    def update(self,c):
        self.c = c
        x = np.append(c[0],c[0,0:ORDER+1])
        y = np.append(c[1],c[1,0:ORDER+1])
        self.t = range(len(x))
        self.t_max = float(len(x) - ORDER - 1)
        self.t_min = 0.0
        self.basis = []
        for i in range(NB_OF_CTRL_POINTS):
            self.basis.append(BSpline.basis_element(self.t[i:i+ORDER+2],False))

        x = np.zeros(NB_OF_CTRL_POINTS)
        y = np.zeros(NB_OF_CTRL_POINTS)
        for i in range(NB_OF_CTRL_POINTS):
            [x[i],y[i]] = self.estimate_with_basis(self.t[i])
        x = np.append(x,x[0])
        y = np.append(y,y[0])
        t2 = range(len(x))
        self.bsplx = CubicSpline(t2,x,bc_type='periodic')
        self.bsply = CubicSpline(t2,y,bc_type='periodic')
        self.derx1 = self.bsplx.derivative(1)
        self.dery1 = self.bsply.derivative(1)
        self.derx2 = self.bsplx.derivative(2)
        self.dery2 = self.bsply.derivative(2)

    def get_basis(self,tk):
        tk = self.modulo_tk(tk)  
        b_term = np.zeros(NB_OF_CTRL_POINTS)
        for i in range(NB_OF_CTRL_POINTS):
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
        for i in range(NB_OF_CTRL_POINTS):
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
        u = np.linspace(self.t_min,self.t_max,50)
        pt = np.zeros((len(u),2))
        pt2 = np.zeros((len(u),2))
        for i in range(len(u)):
            pt[i] = self.estimate(u[i])
            pt2[i] = self.estimate_with_basis(u[i])
        plt.plot(pt[:,0],pt[:,1])
        #plt.plot(pt2[:,0],pt2[:,1],'.r')
        if plot_ctrl_pts:
            plt.plot(self.c[0],self.c[1])


def init_SDM(points):
    P = np.zeros((NB_OF_CTRL_POINTS,2))
    max_x = np.max(points[:,0])
    min_x = np.min(points[:,0])
    max_y = np.max(points[:,1])
    min_y = np.min(points[:,1])
    radius = np.max([max_x-min_x,max_y-min_y])/2 + 10
    center= np.array([min_x+radius-10,min_y+radius-10])
    n = NB_OF_CTRL_POINTS
    P = np.zeros((n,2))
    for i in range(n):
        P[i] = [radius*np.cos(i*2*math.pi/n)+center[0],radius*np.sin(i*2*math.pi/n)+center[1]]
    return P


def find_tk_foot_point(bspl,point):
    # Gradient descent
    speed_factor = 0.01
    epsilon_grad = 0.1
    epsilon_deri = 0.01
    x = 0
    iter_nb = 0
    max_iter_nb = 100
    stop = False
    while not stop:
        y1 = utility.euclidian_dist(bspl.estimate(x),point)
        y2 = utility.euclidian_dist(bspl.estimate(x+epsilon_deri),point)
        grad = (y2-y1)/epsilon_deri
        step = (-grad*speed_factor)
        iter_nb += 1
        if abs(grad) <= epsilon_grad or iter_nb > max_iter_nb:
            stop = True
        else:
            x += step
    tk = bspl.modulo_tk(x)
    return tk
    

def squared_dist(esd,const,dist,rad,Ta_tk,No_tk,tk,neigh,bspl,point):
    # Basis elements
    b_terms = bspl.get_basis(tk)
    #print(tk)
    #print(b_terms)
    #print()

    """
    print('Pt: ',point)
    print('    Neigh: ',neigh)
    print('    Tan: ',Ta_tk)
    print('    Nor: ',No_tk)
    print('    Dist: ',dist)
    print('    Rad: ',rad)
    print()
    """
    # SDM
    if dist >= 0:
    #if dist >= 0 and dist < rad:
        for i in range(NB_OF_CTRL_POINTS):
            for j in range(NB_OF_CTRL_POINTS):
                temp = 2*b_terms[i]*b_terms[j]
                esd[i+0][j+0]                                 += temp*(No_tk[0]**2)    
                esd[i+0][j+NB_OF_CTRL_POINTS]                 += temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+0]                 += temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+NB_OF_CTRL_POINTS] += temp*(No_tk[1]**2)
            const[i+0]                 -= 2*b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+NB_OF_CTRL_POINTS] -= 2*b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    elif dist < 0:
        for i in range(NB_OF_CTRL_POINTS):
            for j in range(NB_OF_CTRL_POINTS):
                temp = 2*b_terms[i]*b_terms[j] 
                esd[i+0][j+0]                                 += temp*(dist/(dist-rad))*(Ta_tk[0]**2) + temp*(No_tk[0]**2)
                esd[i+0][j+NB_OF_CTRL_POINTS]                 += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+0]                 += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+NB_OF_CTRL_POINTS] += temp*(dist/(dist-rad))*(Ta_tk[1]**2) + temp*(No_tk[1]**2)
            const[i+0]                 += 2*(dist/(dist-rad))*b_terms[i]*Ta_tk[0]*np.sum((neigh-point)*Ta_tk) + 2*b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+NB_OF_CTRL_POINTS] += 2*(dist/(dist-rad))*b_terms[i]*Ta_tk[1]*np.sum((neigh-point)*Ta_tk) + 2*b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    else:
        print("Error rad > dist: ",dist,'>',rad)
        plt.show()
        exit()
    return esd,const

def iter_SDM(points,bspl):
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
        K_tk = rad[i]*No_tk[i]
        linear_comb = (points[i]-neigh[i])/K_tk
        # Distance
        dist[i] = utility.euclidian_dist(neigh[i],points[i])
        if linear_comb[np.argmax(np.abs(linear_comb))] < 0:
            dist[i] *= -1

    esd = np.zeros((2*NB_OF_CTRL_POINTS,2*NB_OF_CTRL_POINTS))
    const = np.zeros(2*NB_OF_CTRL_POINTS)
    for i in range(len(points)):
        esd,const = squared_dist(esd,const,dist[i],rad[i],Ta_tk[i],No_tk[i],tk[i],neigh[i],bspl,points[i])
    esd *= 0.5
    const *= 0.5
    D = lstsq(esd,const)[0] # Solve Ax = b 
        
    for i in range(NB_OF_CTRL_POINTS):
        bspl.c[0][i] += D[i]
        bspl.c[1][i] += D[NB_OF_CTRL_POINTS+i]
    
    print(const)
    print()
    print(esd)
    print()
    print(D)

    bspl.update(bspl.c)
    err = 0
    for i in range(len(points)):
        err += (np.sum((neigh[i]-points[i])*No_tk))**2
    print(err/len(points))
    return bspl

 
def SDM_algorithm(points):
    P = init_SDM(points)
    
    # Evaluate bspline
    bspl = bspline(P)
    bspl.plot_curve(True)
    #plt.show()

    for i in range(1):
        bspl = iter_SDM(points,bspl)
        bspl.plot_curve(True)
        #plt.show()


def contour():
    datas = utility.mergedPointsXY
    points = np.zeros((len(datas),2))
    for i in range(len(datas)):
        points[i] = [datas[i].x,datas[i].y]
    
    SDM_algorithm(points)
