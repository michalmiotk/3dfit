import open3d as o3d
import numpy as np
A = 1
B = 0.4
C = 0.3
D = 0
array = []
for x in range(-10,10):
    for y in range(-10,10):
        z = (-D-A*x-B*y)/C
        array.append([x,y,z])

array_points = np.array(array)
center = np.sum(array_points, axis=0)
transformed_points = array_points-center
x,y,z = transformed_points[:,0],transformed_points[:,1],transformed_points[:,2]
D = np.sum(x*x)*np.sum(y*y) - np.sum(x*y)*np.sum(x*y)
a = (np.sum(y*z)*np.sum(x*y) - np.sum(x*z)*np.sum(y*y))/D
b = (np.sum(x*y)*np.sum(x*z) - np.sum(x*x)*np.sum(y*z))/D
print(a,b,D)

'''
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(array))
o3d.visualization.draw_geometries([pcd])
'''