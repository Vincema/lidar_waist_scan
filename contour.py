import utility
import constants
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline, splev, splprep, splrep, CubicSpline
from scipy.linalg import solve,lstsq
from scipy.spatial import distance

NB_OF_CTRL_POINTS = 5
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

        # Sample some values
        self.sample_t = np.linspace(self.t_min,self.t_max,300)
        self.sample_values = np.zeros((len(self.sample_t),2))
        for i in range(len(self.sample_t)):
            self.sample_values[i] = self.estimate(self.sample_t[i])

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
    radius = np.max([max_x-min_x,max_y-min_y])/2 + 50
    center= np.array([min_x+radius-50,min_y+radius-50])
    n = NB_OF_CTRL_POINTS
    P = np.zeros((n,2))
    for i in range(n):
        P[i] = [radius*np.cos(i*2*math.pi/n)+center[0],radius*np.sin(i*2*math.pi/n)+center[1]]
    return P


def find_tk_foot_point(bspl,point):
    dist = np.zeros(len(bspl.sample_t))
    for i in range(len(bspl.sample_t)):
        dist[i] = utility.euclidian_dist(bspl.sample_values[i],point)
    tk = bspl.sample_t[np.argmin(dist)]
    return tk


def squared_dist(esd,const,dist,rad,Ta_tk,No_tk,tk,neigh,bspl,point):
    # Basis elements
    b_terms = bspl.get_basis(tk)

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
                temp = b_terms[i]*b_terms[j]
                esd[i+0][j+0]                                 += temp*(No_tk[0]**2)    
                esd[i+0][j+NB_OF_CTRL_POINTS]                 += temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+0]                 += temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+NB_OF_CTRL_POINTS] += temp*(No_tk[1]**2)
            const[i+0]                 -= b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+NB_OF_CTRL_POINTS] -= b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    elif dist < 0:
        for i in range(NB_OF_CTRL_POINTS):
            for j in range(NB_OF_CTRL_POINTS):
                temp = b_terms[i]*b_terms[j] 
                esd[i+0][j+0]                                 += temp*(dist/(dist-rad))*(Ta_tk[0]**2) + temp*(No_tk[0]**2)
                esd[i+0][j+NB_OF_CTRL_POINTS]                 += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+0]                 += temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+NB_OF_CTRL_POINTS] += temp*(dist/(dist-rad))*(Ta_tk[1]**2) + temp*(No_tk[1]**2)
            const[i+0]                 -= (dist/(dist-rad))*b_terms[i]*Ta_tk[0]*np.sum((neigh-point)*Ta_tk) + 2*b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+NB_OF_CTRL_POINTS] -= (dist/(dist-rad))*b_terms[i]*Ta_tk[1]*np.sum((neigh-point)*Ta_tk) + 2*b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    else:
        print("Error rad > dist: ",dist,'>',rad)
        plt.show()
        exit()

    #c0x =   
    #c1x = 
    #plt.plot([neigh[0],neigh[0]+c0x],[neigh[1],neigh[1]+c1x],'k')
    return esd,const

def mark_zeros_line(esd,tresh):
    zeros = []
    for i in range(NB_OF_CTRL_POINTS*2):
        all_zeros = True
        for j in range(NB_OF_CTRL_POINTS*2):
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

def apply_zeros_constraints(esd,const,zeros):
    for i in range(NB_OF_CTRL_POINTS*2):
        if zeros[i]:
            for j in range(NB_OF_CTRL_POINTS*2):
                esd[i][j] = 0.0
                esd[j][i] = 0.0
            esd[i][i] = 1.0
            const[i] = 0.0
    return esd,const

def move_ctrl_points(P,D,zeros):
    for i in range(NB_OF_CTRL_POINTS):
        if not zeros[i]:
            P[0][i] += D[i]
        if not zeros[NB_OF_CTRL_POINTS+i]:
            P[1][i] += D[NB_OF_CTRL_POINTS+i]

    for i in range(NB_OF_CTRL_POINTS):
        if zeros[i]:
            j = (i-1)%NB_OF_CTRL_POINTS
            diff1 = 1
            while zeros[j] == True:
                j = (j-1)%NB_OF_CTRL_POINTS
                diff1 += 1 
            x1 = P[0][j]
            j = (i+1)%NB_OF_CTRL_POINTS
            diff2 = 1
            while zeros[j] == True:
                j = (j+1)%NB_OF_CTRL_POINTS
                diff2 += 1 
            x2 = P[0][j]
            x_mid = x1 + (x2-x1)/(diff1+diff2)
            x = x_mid + (P[0][i]-x_mid)/2
            P[0][i] = x
    for i in range(NB_OF_CTRL_POINTS):
        if zeros[i+NB_OF_CTRL_POINTS]:
            j = (i-1)%(NB_OF_CTRL_POINTS) 
            diff1 = 1
            while zeros[j+NB_OF_CTRL_POINTS] == True:
                j = (j-1)%NB_OF_CTRL_POINTS
                diff1 += 1 
            y1 = P[1][j]
            j = (i+1)%NB_OF_CTRL_POINTS
            diff2 = 1
            while zeros[j+NB_OF_CTRL_POINTS] == True:
                j = (j+1)%NB_OF_CTRL_POINTS
                diff2 += 1 
            y2 = P[1][j]
            y_mid = y1 + (y2-y1)/(diff1+diff2)
            y = y_mid + (P[1][i]-y_mid)/2
            P[1][i] = y
    return P
            
            

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
        linear_comb = (points[i]-neigh[i])*No_tk[i]
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

    zeros = mark_zeros_line(esd,0.5)
    esd,const = apply_zeros_constraints(esd,const,zeros)
    
    D = lstsq(esd,const)[0]

    move_ctrl_points(bspl.c,D,zeros)

    #for i in range(NB_OF_CTRL_POINTS):
    #    bspl.c[0][i] += D[i]
    #    bspl.c[1][i] += D[NB_OF_CTRL_POINTS+i]
    
    print(const)
    print()
    print(esd)
    print()
    print(D)
    for i in range(NB_OF_CTRL_POINTS*2):
        print(np.max((esd[i,:])))

    bspl.update(bspl.c)

    return bspl


def SDM_algorithm(points):
    P = init_SDM(points)
    
    # Evaluate bspline
    bspl = bspline(P)
    bspl.plot_curve(True)
    #plt.show()

    for i in range(10):
        bspl = iter_SDM(points,bspl)
        #bspl.plot_curve(True)
        #plt.show()
    bspl.plot_curve(True)

def contour():
    datas = utility.mergedPointsXY
    points = np.zeros((len(datas),2))
    for i in range(len(datas)):
        points[i] = [datas[i].x,datas[i].y]
    
    SDM_algorithm(points)
