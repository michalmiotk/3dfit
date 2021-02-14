import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

assumed_D = -1.4

def filter_points_z(points, min, max):
    pcd_points = []
    for i in range(points.shape[0]):
        if min < points[i,2] < max:
            pcd_points.append(list(points[i])) 
    return pcd_points

def get_points_from_file():
    pcd_read = o3d.io.read_point_cloud("mapcloud_low_res.pcd")
    pcd_read_points = np.array(pcd_read.points)
    return filter_points_z(pcd_read_points, assumed_D, -0.5)

def put_points_on_plane(point3D,A,B,C,D):
    x,y,z = point3D
    length_vector = np.sqrt(A**2+B**2+C**2)
    A_norm, B_norm, C_norm, D_norm  = map(lambda x: x/length_vector, [A,B,C,D])
    dist = (A_norm*x+B_norm*y+C_norm*z+D_norm)/np.sqrt(A_norm**2+B_norm**2+C_norm**2)
    return x-A_norm*dist, y-B_norm*dist,z-C_norm*dist

def create_plane(A,B,C,D=0):
    array = []
    for x in range(-10,10):
        for y in range(-10,10):
            z = (-D-A*x-B*y)/C
            array.append([x,y,z])
    return array

def filter_image_by_points_nr(image, min_point_nr):
    return np.where(image>min_point_nr, image, 0)

def cloud_to_image(A, B, C, D, array):

    mapped_3Dpoints = [*map(lambda point: put_points_on_plane(point, A,B,C,D), array)]
    mapped_2Dpoints = [*map(lambda point: [point[0], point[1]], mapped_3Dpoints)]
    np_mapped_2Dpoints = np.array(mapped_2Dpoints)
    max_x, max_y, min_x, min_y = np.max(np_mapped_2Dpoints[:,0]), np.max(np_mapped_2Dpoints[:,1]), np.min(np_mapped_2Dpoints[:,0]), np.min(np_mapped_2Dpoints[:,1])
    print('max_x, max_y, min_x, min_y', max_x, max_y, min_x, min_y)
    pixel_per_meter = 5
    image = np.zeros((int(pixel_per_meter*(max_y-min_y)), int(pixel_per_meter*(max_x-min_x))))
    for x,y in mapped_2Dpoints:
        x_pix, y_pix = int(pixel_per_meter*(x-min_x)), int(pixel_per_meter*(y-min_y))
        if not 0<=x_pix<image.shape[1]:
            print('x_pix',x_pix, 'powinno',  pixel_per_meter*min_x, pixel_per_meter*max_x)
            continue
        if not 0<=y_pix<image.shape[0]:
            print('y_pix',y_pix, 'powinno',  pixel_per_meter*min_y, pixel_per_meter*max_y)
            continue

        image[image.shape[0]-1-y_pix][x_pix] += 1
    return image

def fit_plane_to_points(array):
    array_points = np.array(array)
    center = np.mean(array_points, axis=0)
    transformed_points = array_points-center
    x,y,z = transformed_points[:,0],transformed_points[:,1],transformed_points[:,2]
    D = np.sum(x*x)*np.sum(y*y) - np.sum(x*y)*np.sum(x*y)
    a = (np.sum(y*z)*np.sum(x*y) - np.sum(x*z)*np.sum(y*y))/D
    b = (np.sum(x*y)*np.sum(x*z) - np.sum(x*x)*np.sum(y*z))/D
    return a,b,1, D
array = get_points_from_file()
a,b,_,D = fit_plane_to_points(array)
print(a,b,D)
plane_points = create_plane(a,b,1, -assumed_D)
image = cloud_to_image(a,b,1, -assumed_D, filter_points_z(np.array(array), assumed_D+0.2, -0.5))
image = filter_image_by_points_nr(image, 2)
merg_array = array + plane_points
plt.imshow(image)
plt.show()

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(merg_array))
o3d.visualization.draw_geometries([pcd])