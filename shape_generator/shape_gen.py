import numpy as np
import matplotlib.pyplot as plt

nb_side_shape = 10
nb_points = 200
stdev_edges_shape = 10
stdev_points = 3
radius_shape = 100

points_circle = np.linspace(0, 2*np.pi, nb_side_shape, endpoint=False) 
circle = [np.cos(points_circle) * radius_shape, np.sin(points_circle) * radius_shape]

shape_coef = np.random.standard_normal(size=(2,nb_side_shape)) * stdev_edges_shape 
shape = circle + shape_coef

pts_x = np.array([])
pts_y = np.array([])
for i in range(nb_side_shape):
    for j in range(int(np.ceil(nb_points/nb_side_shape))):
        rand_bias = np.random.standard_normal(2)* stdev_points
        pts_x = np.append(pts_x, rand_bias[0] + shape[0, i] + (shape[0,(i+1)%nb_side_shape] - shape[0, i]) * j / (np.ceil(nb_points/nb_side_shape)))
        pts_y = np.append(pts_y, rand_bias[1] + shape[1, i] + (shape[1,(i+1)%nb_side_shape] - shape[1, i]) * j / (np.ceil(nb_points/nb_side_shape)))

f = open(r"C:\Users\Toshiba\Desktop\Temp\datas.txt",'w')
for i in range(nb_points):
    f.write(str(pts_x[i]) + ' ' + str(pts_y[i]) + '\n')
f.close()

plt.plot(np.append(shape[0], shape[0,0]), np.append(shape[1], shape[1,0]),'-g')
plt.plot(pts_x, pts_y, '.k', ms=3)
plt.gca().set_aspect('equal')
plt.show()
