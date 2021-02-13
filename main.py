import numpy as np
import open3d as o3d
import math

def get_complex_deriv(f_x, df_x, g_x, dg_x):
    return (f_x*dg_x-df_x*g_x)/(g_x*g_x)

def common_dg_x(letter, A,B,C):
    if letter == 'A':
        return 2*A
    if letter == 'B':
        return 2*B 
    if letter == 'C':
        return 2*C 
    if letter == 'D':
        return 0

def common_df_x(letter, A,B,C,D, x,y,z):
    if letter == 'A':
        return 2*(A*x+B*y+C*z+D)*x
    if letter == 'B':
        return 2*(A*x+B*y+C*z+D)*y
    if letter == 'C':
        return 2*(A*x+B*y+C*z+D)*z
    if letter == 'D':
        return 2*(A*x+B*y+C*z+D)

def common_g_x(A,B,C):
    return pow(A,2)+pow(B,2) + pow(C,2)

def common_f_x(A,B,C,D,x,y,z):
    return pow(A*x+B*y+C*z+D,2)

def get_deriv_letter(letter, A,B,C,D,x,y,z):
    f_x = common_f_x(A,B,C,D,x,y,z)
    df_x = common_df_x(letter, A,B,C,D, x,y,z)
    g_x = common_g_x(A,B,C)
    dg_x = common_dg_x(letter,A,B,C)
    return get_complex_deriv(f_x, df_x, g_x, dg_x)

def get_error(A,B,C, D, x,y,z):
    return abs(A*x+B*y+C*z+D)/math.sqrt(pow(A,2)+pow(B,2)+pow(C,2))

A = 1
B = 1
C = 1
D = 0
array = []
for x in range(-10,10):
    for y in range(-10,10):
        z = (-D-A*x-B*y)/C
        array.append([x,y,z])
'''
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(array))
o3d.visualization.draw_geometries([pcd])
'''

A = 0.7
B = 0.8
C = 0.9
D = 0
lr = 0.000001
epoch_nr = 10
for epoch in range(epoch_nr):
    sum_err = 0
    A_corr, B_corr, C_corr, D_corr = 0,0,0,0
    for x,y,z in array:
        sum_err += get_error(A,B,C,D,x,y,z)
        A_corr += get_deriv_letter('A', A,B,C,D,x,y,z)
        B_corr += get_deriv_letter('B', A,B,C,D,x,y,z)
        C_corr += get_deriv_letter('C', A,B,C,D,x,y,z)
        D_corr += get_deriv_letter('D', A,B,C,D,x,y,z)
        #print("corrections", A_corr,B_corr, C_corr, D_corr)
    A -= lr*A_corr
    B -= lr*B_corr
    C -= lr*C_corr
    D -= lr*D_corr
    print(sum_err, A,B,C,D)
'''
#pochodna D

#pcd = o3d.geometry.PointCloud()
#pcd.points = o3d.utility.Vector3dVector(np.array(array))

pcd_read = o3d.io.read_point_cloud("/home/m/Pobrane/mapcloud_low_res.pcd")
pcd_read_points = np.array(pcd_read.points)

pcd_points = []
for i in range(pcd_read_points.shape[0]):
    if -1.5 < pcd_read_points[i,2] < 1.5:
        pcd_points.append(list(pcd_read_points[i])) 
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(pcd_points))
o3d.visualization.draw_geometries([pcd])
'''