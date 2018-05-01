import utility
import constants
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline, splev, splprep, CubicSpline
from scipy.linalg import solve

NB_OF_CTRL_POINTS = 7
ORDER = 3

def compute_bspline(P):
    pts = np.array(P)
    x = pts[:,0]
    y = pts[:,1]
    x = np.append(x,pts[0,0])   # To close the loop
    y = np.append(y,pts[0,1])   # To close the loop
    tck,u = splprep([x,y],k=ORDER,s=0,per=True,quiet=True)
    return tck

def interp_bspline(tck,u=[],der=0):
    if u == []:
        u = np.linspace(0,1,100,endpoint=True)
        interp_values = splev(u,tck,der=der)
    elif isinstance(u,(float,int)):
        u = u%1
    else:
        for i in u:
            i = i%1
    interp_values = splev(u,tck,der=der)
    return interp_values


def init_SDM(points):
    P = np.zeros((NB_OF_CTRL_POINTS,2))
    max_x = np.max(points[:,0])
    min_x = np.min(points[:,0])
    max_y = np.max(points[:,1])
    min_y = np.min(points[:,1])
    radius = np.max([max_x-min_x,max_y-min_y])/2 + 20
    center= np.array([min_x+radius-20,min_y+radius-20])
    n = NB_OF_CTRL_POINTS-ORDER
    P = np.zeros((n,2))
    for i in range(n):
        P[i] = [radius*np.cos(i*2*math.pi/n)+center[0],radius*np.sin(i*2*math.pi/n)+center[1]]
    return P


def find_tk_foot_point(bspl,point):
    # Gradient descent
    speed_factor = 0.0001
    epsilon_grad = 0.1
    epsilon_deri = 0.001
    x = 0
    iter_nb = 0
    max_iter_nb = 100
    stop = False
    while not stop:
        y1 = utility.euclidian_dist(interp_bspline(bspl,x),point)
        y2 = utility.euclidian_dist(interp_bspline(bspl,x+epsilon_deri),point)
        grad = (y2-y1)/epsilon_deri
        step = (-grad*speed_factor)
        iter_nb += 1
        if abs(grad) <= epsilon_grad or iter_nb > max_iter_nb:
            stop = True
        else:
            x += step
    tk = x%1
    return tk
    

def squared_dist(dist,rad,Ta_tk,No_tk,tk,neigh,bspl,point):
    
    u = np.linspace(0,1,100)
    pt = np.zeros((100,2))
    for i in range(len(u)):
        #bspl[1][0][4] = 150
        b_terms = get_basis_elements(bspl,u[i])
        #print(b_terms)
        for j in range(NB_OF_CTRL_POINTS):
            pt[i] += [b_terms[j]*bspl[1][0][j],b_terms[j]*bspl[1][1][j]]
    #plt.plot(bspl[1][0][:],bspl[1][1][:])
    plt.plot(pt[:,0],pt[:,1])
    plt.show()


    # Basis elements
    b_terms = get_basis_elements(bspl,tk)
    print(b_terms)
    print()
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
    esd = np.zeros((NB_OF_CTRL_POINTS*2,NB_OF_CTRL_POINTS*2))
    const = np.zeros(NB_OF_CTRL_POINTS*2)
    #if dist >= 0 and dist < rad:
    if dist >= 0:
        for i in range(NB_OF_CTRL_POINTS):
            for j in range(NB_OF_CTRL_POINTS):
                temp = 2*b_terms[i]*b_terms[j]
                esd[i+0][j+0]                                 = temp*(No_tk[0]**2)    
                esd[i+0][j+NB_OF_CTRL_POINTS]                 = temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+0]                 = temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+NB_OF_CTRL_POINTS] = temp*(No_tk[1]**2)
            const[i+0]                 = 2*b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+NB_OF_CTRL_POINTS] = 2*b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    elif dist < 0:
        for i in range(NB_OF_CTRL_POINTS):
            for j in range(NB_OF_CTRL_POINTS):
                temp = 2*b_terms[i]*b_terms[j] 
                esd[i+0][j+0]                                 = temp*(dist/(dist-rad))*(Ta_tk[0]**2) + temp*(No_tk[0]**2)
                esd[i+0][j+NB_OF_CTRL_POINTS]                 = temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+0]                 = temp*(dist/(dist-rad))*Ta_tk[0]*Ta_tk[1] + temp*No_tk[0]*No_tk[1]
                esd[i+NB_OF_CTRL_POINTS][j+NB_OF_CTRL_POINTS] = temp*(dist/(dist-rad))*(Ta_tk[1]**2) + temp*(No_tk[1]**2)
            const[i+0]                 = 2*(dist/(dist-rad))*b_terms[i]*Ta_tk[0]*np.sum((neigh-point)*Ta_tk) + 2*b_terms[i]*No_tk[0]*np.sum((neigh-point)*No_tk)
            const[i+NB_OF_CTRL_POINTS] = 2*(dist/(dist-rad))*b_terms[i]*Ta_tk[1]*np.sum((neigh-point)*Ta_tk) + 2*b_terms[i]*No_tk[1]*np.sum((neigh-point)*No_tk)
    else:
        print("Error rad > dist: ",dist,'>',rad)
        plt.show()
        exit()
    return esd,const


def update_curve(basis,P):
    u = np.linspace(0,1,NB_OF_CTRL_POINTS)
    new_P = np.zeros((NB_OF_CTRL_POINTS,2))
    for i in range(NB_OF_CTRL_POINTS):
        for j in range(NB_OF_CTRL_POINTS):
            new_P[i] += np.array([basis[j](u[i]) * P[j][0], basis[j](u[i]) * P[j][1]])

    updated_bspl = compute_bspline(new_P)
    return updated_bspl,new_P


def iter_SDM(P,points,bspl):
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
        neigh[i] = interp_bspline(bspl,tk[i])
        # Derivatives
        der_1 = interp_bspline(bspl,tk[i],der=1)
        der_2 = interp_bspline(bspl,tk[i],der=2)
        # Unit tangeant vector and unit normal vector at the point neigh
        Ta_tk[i] = np.array([der_1[0],der_1[1]])
        Ta_tk[i] = Ta_tk[i]/utility.euclidian_dist([0,0],Ta_tk[i])
        No_tk[i] = np.array([der_2[0],der_2[1]])
        No_tk[i] = No_tk[i]/utility.euclidian_dist([0,0],No_tk[i])
        # Radius
        rad[i] = abs(((der_1[0]**2 + der_1[1]**2)**(3/2))/(der_1[0]*der_2[1]-der_2[0]*der_1[1]))
        K_tk = rad[i]*No_tk[i]
        linear_comb = (points[i]-neigh[i])/No_tk[i]
        # Distance
        dist[i] = utility.euclidian_dist(neigh[i],points[i])
        if linear_comb[0] < 0 and linear_comb[1] < 0:
            dist[i] *= -1

    fsd = np.zeros((2*NB_OF_CTRL_POINTS,2*NB_OF_CTRL_POINTS))
    const = np.zeros(2*NB_OF_CTRL_POINTS)
    for i in range(len(points)):
        out = squared_dist(dist[i],rad[i],Ta_tk[i],No_tk[i],tk[i],neigh[i],bspl,points[i])
        fsd += 0.5*out[0] 
        const += 0.5*out[1]
    D = solve(fsd,const)

    new_c = bspl[1]
    for i in range(NB_OF_CTRL_POINTS):
        new_c[0][i] += D[i]
        new_c[1][i] += D[NB_OF_CTRL_POINTS+i]

    new_bspl = np.array([bspl[0],new_c,bspl[2]]) 
    print(D)
    print()
    print(fsd)
    print()
    print(const)

    u = np.linspace(0,1,100)
    pt = np.zeros((100,2))
    for i in range(len(u)):
        basis = get_basis_elements(bspl,u[i])
        for j in range(NB_OF_CTRL_POINTS):
            pt[i] += np.array([basis[j] * new_c[0][j], basis[j] * new_c[1][j]])
    plt.plot(pt[:,0],pt[:,1])
    #plt.plot(P[:,0],P[:,1])
    plt.show()
    
    updated_bspl,new_P = update_curve(basis,P)

    return updated_bspl,new_P
 

def get_basis_elements(bspl,tk):
    # Basis functions
    bterm = np.zeros(NB_OF_CTRL_POINTS)
    t = bspl[0]
    c = bspl[1]
    order = bspl[2]
    for i in range(NB_OF_CTRL_POINTS):
        temp_c = np.zeros((2,NB_OF_CTRL_POINTS))
        temp_c[0][i] = 1
        temp_bspl = np.array([t,temp_c,order])
        bterm[i] = interp_bspline(temp_bspl,tk)[0]
    return bterm


def SDM_algorithm(points):
    P = init_SDM(points)
    
    # Evaluate bspline
    bspl = compute_bspline(P)
    
    out = interp_bspline(bspl)
    plt.plot(out[0],out[1])
    plt.plot(P[:,0],P[:,1])

    for i in range(2):
        bspl,P = iter_SDM(P,points,bspl)
        out = interp_bspline(bspl)
        plt.plot(out[0],out[1])
        plt.plot(P[:,0],P[:,1])
        plt.show()


def contour():
    datas = utility.mergedPointsXY
    points = []
    for i in range(len(datas)):
        points.append([datas[i].x,datas[i].y])
    points = np.array(points)

    SDM_algorithm(points)
