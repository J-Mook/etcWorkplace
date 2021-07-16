from numpy.core.records import array
from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math, random
from plyfile import PlyElement, PlyData
import time
PI = 3.14159265358979

##initialize
Source_point = [500, 500, 400] #lidar location (width ,depth ,height)
Source_target = [0, 0, 100]
camera_moving_mount = 1
mode_select = 'pinking'

#Lidar Mode
Angular_Resolution = [math.radians(1), math.radians(0.2)] #[ vertical , horizontal ]
model_select = 'm'
noise_mode = True

#Pinking Mode
gaussian_mode = True

mode_data = {'lidar' : True , 'pinking' : False}
models_data = {'xs' : [[106, 118, 133],[161,181,205],[70,78,88],[0.035]], 
                's' : [[343, 360, 382],[384,442,520],[237,272,319],[0.050]], 
                'm' : [[317, 590, 826],[458, 650, 1118],[292,404,686],[0.100]], 
                'l' : [[600, 1082, 1644],[870, 1239, 2150],[557, 772, 1326],[0.200]]} #[[width],[lenth],[height],[noise]]

def make_pcd_spr2pnt(pSource, angle_sqr, sphere_redius):
    # point to surface
    pcd = []
    min_angle_xy, max_anlge_xy, min_angle_z, max_anlge_z = setting_ROI_angle(pSource, Source_target)
    seperate_xy = int((max_anlge_xy - min_angle_xy) / angle_sqr[1])
    seperate_z = int((max_anlge_z - min_angle_z) / angle_sqr[0])

    
    for i in tqdm(range(0, seperate_z), leave = False, position = 2):
        for j in range(0, seperate_xy):
            pTarget = creat_sphere(sphere_redius, (i * angle_sqr[0]) + min_angle_z, (j * angle_sqr[1]) + min_angle_xy, pSource)
            try:
                pointsIntersection = caster.castRay(pSource, pTarget)
                if check_fov(pSource, pointsIntersection[0]):
                    if noise_mode:
                        noise_x = pointsIntersection[0][0] + random.gauss(0, models_data[model_select][3][0]/3)
                        noise_y = pointsIntersection[0][1] + random.gauss(0, models_data[model_select][3][0]/3)
                        noise_z = pointsIntersection[0][2] + random.gauss(0, models_data[model_select][3][0]/3)
                        
                        noise_point = tuple((noise_x, noise_y, noise_z))
                        pcd.append(noise_point)
                    else:
                        pcd.append(pointsIntersection[0])
            except:
                pass
    return pcd, seperate_xy * seperate_z
    
def make_pcd_ply2pnt(pSource, ply_file_path):
    # point to surface
    pcd = []
    ply = o3d.io.read_point_cloud(ply_file_path)
    ply_arr = np.array(ply.points)
    mesh_error_correction = pSource / np.linalg.norm(pSource)

    for suface_pcd in tqdm(ply_arr, leave = False, position = 2):
        try:
            pointsIntersection = caster.castRay(pSource, (suface_pcd + mesh_error_correction)) # allow error
            # pointsIntersection = caster.castRay(pSource, suface_pcd) # not allow error
            # if check_fov(pSource, pointsIntersection[0]):
            if len(pointsIntersection) <= 0:
                pcd.append(tuple(suface_pcd))
        except:
            pass

    return pcd, len(ply_arr)
    
def setting_ROI_angle(start_point, end_point):
    source_angle_xy = math.atan2(0 - start_point[1], 0 - start_point[0])
    # source_angle_z = cal_angle([start_point[0], start_point[1], 0], [ 0 - start_point[0], 0 - start_point[1], start_point[2] + target_z])
    source_angle_z = cal_angle([start_point[0],start_point[1], 0], [start_point[0] - end_point[0],
                                 start_point[1] - end_point[1], start_point[2] - end_point[2]])
    
    # xy_min = source_angle_xy - PI/4
    # xy_max = source_angle_xy + PI/4
    # z_min = source_angle_z - PI/8 + PI/2
    # z_max = source_angle_z + PI/8 + PI/2
    xy_angle = math.atan(abs(((models_data[model_select][0][2] - models_data[model_select][0][0]) / 2 ) / (models_data[model_select][1][2] - models_data[model_select][1][0])))
    xy_min = source_angle_xy - xy_angle
    xy_max = source_angle_xy + xy_angle
    z_anlge = math.atan(abs(((models_data[model_select][2][2] - models_data[model_select][2][0]) / 2 ) / (models_data[model_select][1][2] - models_data[model_select][1][0])))
    z_min = source_angle_z - z_anlge + PI/2
    z_max = source_angle_z + z_anlge + PI/2
    
    return xy_min, xy_max, z_min, z_max


def creat_sphere(r, phi, theta, source_point):
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

def cal_angle(v, w):
    return np.arccos(np.dot(v,w)/(np.linalg.norm(v)*np.linalg.norm(w)))

def find_sphere_redius(file, point):
    mesh = o3d.io.read_triangle_mesh(file)
    mesh = mesh.sample_points_poisson_disk(1000)
    # o3d.visualization.draw_geometries([mesh])
    mesh = np.array(mesh.points)
    longest_distance = 0
    for k in mesh:
        pnt2src_dist = math.dist(k,point)
        longest_distance = max(pnt2src_dist,longest_distance)

    return longest_distance

def camera_move(start_point, moving_mount):
    move_redius = math.dist(start_point,[0,0,0])
    fist_angle = math.atan2(start_point[1], start_point[0])
    move_angle = 2 * PI * moving_mount / camera_moving_mount + fist_angle
    x = move_redius * math.cos(move_angle)
    y = move_redius * math.sin(move_angle)
    z = start_point[2]

    return [x, y, z]

def check_fov(source_point, check_point):
    if math.dist(source_point, check_point) < models_data[model_select][1][0] * 0.9:
        return False
    else:
        return True
    # return True


def calc_projection(a, b):
    # P = np.outer(a, a) / a.dot(a)
    # return P.dot(b.T)
    return abs(np.linalg.norm((np.dot(a, b) / np.dot(b, b)) * b))

    
##search stl file data
input_data_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
save_ply_folder_path = os.path.join(input_data_folder_path, "ply")

if mode_data[mode_select]:
    stl_file_name = "TestSpecimenAssy.stl"
    stl_file = os.path.join(input_data_folder_path, stl_file_name)
    caster = rayCaster.fromSTL(stl_file, scale = 1)
else:
    stl_file_name = "TestSpecimenAssy.stl"
    ply_file_name = "test_specimen_assy_poisson_sampling_66593pts.ply"
    # ply_file_name = "test_specimen_assy_montecarlo_sampling_50000pts.ply"
    input_ply_file = os.path.join(input_data_folder_path, ply_file_name)
    stl_file = os.path.join(input_data_folder_path, stl_file_name)
    caster = rayCaster.fromSTL(stl_file, scale = 1)

start = time.time()
##scanning multiple location
for move_num in tqdm(range(camera_moving_mount),leave = False, position = 0):
    
    pcd_list = []
    pcd = o3d.geometry.PointCloud()

    if mode_data[mode_select]:   ##LidarMode
        if camera_moving_mount != 1:
            now_point = camera_move(Source_point, move_num)
            # redius = find_sphere_redius(stl_file, now_point)
            redius = models_data[model_select][1][2]
            pcd_list, pnt_mount = make_pcd_spr2pnt(now_point, Angular_Resolution, redius)
        else:
            # redius = find_sphere_redius(stl_file, Source_point)
            redius = models_data[model_select][1][2]
            pcd_list, pnt_mount = make_pcd_spr2pnt(Source_point, Angular_Resolution, redius)
    
    else:   ##PinkingMode
        input_ply_file = os.path.join(input_data_folder_path, ply_file_name)
        pcd_list, pnt_mount = make_pcd_ply2pnt(Source_point, input_ply_file)

    ## scanning data convert to pointcloud data
    pcd_array = np.asarray(pcd_list, dtype=np.float32)
    pcd.points = o3d.utility.Vector3dVector(pcd_array)
    # pcd = pcd.voxel_down_sample(voxel_size=0.001)

    ##scanning data visulaization
    # print(len(np.array(pcd.points)))
    if pcd_array.size != 0:
        # # o3d.visualization.draw_geometries([pcd])
        # ply_name = stl_file_name.split(".")[0]+ "_" +format(pnt_mount, '04') + ".ply"
        # save_path = os.path.join(save_ply_folder_path, ply_name)
        # save_float_ply(pcd_array, save_path)
        pass
    else:
        print("!!!pointcloud is empty!!!")

# print(pnt_mount)
# print(time.time() - start)
# print(len(np.array(pcd.points)))
o3d.visualization.draw_geometries([pcd])
