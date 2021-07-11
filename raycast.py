from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math
# from pycaster.test import test_all

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

def make_pcd_srf2srf():
    # surface to surface
    for i in tqdm(range(-500,500,point_dencity)):
        for j in range(-500,500,point_dencity):
            pSource = [i, -1000.0, j]
            pTarget = [i, 1000.0, j]
            pSource = np.dot(rotation_matrix(axis, theta), pSource)
            pTarget = np.dot(rotation_matrix(axis, theta), pTarget)
            
            try:
                pointsIntersection = caster.castRay(pSource, pTarget)
                pcd_list.append(pointsIntersection[0])
                # pcd_list.append(pointsIntersection[1])
                
            except:
                pass

def make_pcd_srf2pnt():
    # point to surface
    pSource = [500, -1000.0, 500]
    for i in tqdm(range(-2000,2000,point_dencity)):
        for j in range(-2000,2000,point_dencity):
            pTarget = [i, 500.0, j]
            # pSource = np.dot(rotation_matrix(axis, theta), pSource)
            # pTarget = np.dot(rotation_matrix(axis, theta), pTarget)
            
            try:
                pointsIntersection = caster.castRay(pSource, pTarget)
                pcd_list.append(pointsIntersection[0])
                # pcd_list.append(pointsIntersection[1])
                
            except:
                pass


stl_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
	
# ply_list = sorted(os.listdir(ply_folder_path))
file_name = "TestSpecimenAssy.stl"
file = os.path.join(stl_folder_path, file_name)

caster = rayCaster.fromSTL(file, scale = 1)
pcd_list = []

point_dencity = 20
axis = [1, 1, 1]
theta = -30

make_pcd_srf2pnt()

pcd_array = np.asarray(pcd_list, dtype=np.float32)

print(pcd_array)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd_array)

# pcd = o3d.io.read_point_cloud(file)
# pcd = pcd.voxel_down_sample(voxel_size=0.001)

o3d.visualization.draw_geometries_with_editing([pcd])