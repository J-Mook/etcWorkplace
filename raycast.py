from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math
from plyfile import PlyElement, PlyData

PI = 3.14159265358979


def make_pcd_spr2pnt(pSource, seperate_n, sphere_redius):
    # point to surface
    source_angle = math.atan2(-1 * pSource[1], -1 * pSource[0])
    min_angle = source_angle - PI/4
    max_anlge = source_angle + PI/4

    for i in tqdm(range(0, seperate_n)):
        for j in range(0, seperate_n):
            # pTarget = creat_spherial(sphere_redius, i * 2 * PI/seperate_n, j * PI/seperate_n, pSource)
            pTarget = creat_spherial(sphere_redius, i * PI/seperate_n, (j * abs(max_anlge - min_angle) / seperate_n) + min_angle, pSource)
            try:
                pointsIntersection = caster.castRay(pSource, pTarget)
                pcd_list.append(pointsIntersection[0])      
            except:
                pass

def creat_spherial(r, phi, theta, source_point):
    x = r * math.sin(phi) * math.cos(theta) + source_point[0]
    y = r * math.sin(phi) * math.sin(theta) + source_point[1]
    z = r * math.cos(phi) + source_point[2]
    return [x, y, z]

def rotation_matrix(axis, theta):
    
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(math.degrees(theta))
    b, c, d = -axis * math.sin(math.degrees(theta))
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def save_float_ply(arr, save_path):
    arr = arr.T
    arr = list(zip(arr[0], arr[1], arr[2]))
    vertex = np.array(arr, dtype=[('x', 'f4'), ('y', 'f4'),('z', 'f4')])
    el = PlyElement.describe(vertex, 'vertex')
    PlyData([el],text=True).write(save_path)

def cal_angle(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = np.linalg.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)

def find_sphere_redius(file):
    mesh = o3d.io.read_triangle_mesh(file)
    mesh = mesh.sample_points_poisson_disk(1000)
    # o3d.visualization.draw_geometries([mesh])
    mesh = np.array(mesh.points)
    longest_distance = 0
    for k in tqdm(range(len(mesh))):
        pnt2src_dist = math.dist(mesh[k],Source_point)
        mesh[k][2] = pnt2src_dist
        longest_distance = max(pnt2src_dist,longest_distance)


    # np.insert(mesh, 0, 3, axis = 1)
    # print(mesh[10])
    # biggest_anlge = 0
    # biggest_anlge2 = 0
    # max_x = max(mesh[:][0])
    # min_x = min(mesh[:][0])
    # max_y = max(mesh[:][1])
    # min_y = min(mesh[:][1])
    # max_z = mesh.sort(key = lambda x : x[:][2]).pop(0)
    # min_z = mesh.sort(key = lambda x : x[:][2]).pop(-1)
    # Source_point_xy = [Source_point[0], Source_point[1]]
    

    # for point in tqdm(mesh): 
    #     angle_point = [point[k] - Source_point[k] for k in range(3)]
    #     Source_anlge_point = [-1 * Source_point[k] for k in range(3)]
    #     biggest_anlge = max(cal_angle(angle_point, Source_anlge_point), biggest_anlge)
    
    return longest_distance
##initialize
pcd_list = []
pcd = o3d.geometry.PointCloud()
Source_point = [200, -1000.0, 400] #lidar location (width ,depth ,height)
seperate_spr = 1000 #liar point amount


##search stl file data
stl_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
save_ply_folder_path = os.path.join(stl_folder_path, "ply")

# stl_list = sorted(os.listdir(stl_folder_path))
# for stl_file_name in stl_list:
stl_file_name = "TestSpecimenAssy.stl"
stl_file = os.path.join(stl_folder_path, stl_file_name)
caster = rayCaster.fromSTL(stl_file, scale = 1)

##scanning
redius = find_sphere_redius(stl_file)
make_pcd_spr2pnt(Source_point, seperate_spr, redius)

## scanning data convert to pointcloud data
pcd_array = np.asarray(pcd_list, dtype=np.float32)
pcd.points = o3d.utility.Vector3dVector(pcd_array)
# pcd = pcd.voxel_down_sample(voxel_size=0.001)

##sanning data visulaization
print(len(np.array(pcd.points)))
if pcd != []:
    o3d.visualization.draw_geometries([pcd])
else:
    print("pointcloud is empty")

# ply = np.asarray(pcd_array.points)
ply_name = stl_file_name.split(".")[0] + ".ply"
save_path = os.path.join(save_ply_folder_path, ply_name)
save_float_ply(pcd_array, save_path)