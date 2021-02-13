import open3d as o3d
import numpy as np
def get_points_from_file():
    pcd_read = o3d.io.read_point_cloud("mapcloud_low_res.pcd")
    pcd_read_points = np.array(pcd_read.points)

    pcd_points = []
    for i in range(pcd_read_points.shape[0]):
        if -1.5 < pcd_read_points[i,2] < -0.5:
            pcd_points.append(list(pcd_read_points[i])) 
    return pcd_points

def create_plane(A,B,C):
    D = 0
    array = []
    for x in range(-10,10):
        for y in range(-10,10):
            z = (-D-A*x-B*y)/C
            array.append([x,y,z])
    return array
array = get_points_from_file()
array_points = np.array(array)
center = np.mean(array_points, axis=0)
transformed_points = array_points-center
x,y,z = transformed_points[:,0],transformed_points[:,1],transformed_points[:,2]
D = np.sum(x*x)*np.sum(y*y) - np.sum(x*y)*np.sum(x*y)
a = (np.sum(y*z)*np.sum(x*y) - np.sum(x*z)*np.sum(y*y))/D
b = (np.sum(x*y)*np.sum(x*z) - np.sum(x*x)*np.sum(y*z))/D
print(a,b,D)
plane_points = create_plane(a,b,1)
merg_array = array + plane_points

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(merg_array))
o3d.visualization.draw_geometries([pcd])
