from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math
# from pycaster.test import test_all

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

def make_pcd_spr2pnt():
    # point to surface
    pSource = [1000, -1000.0, 500] #in coordinate (width ,depth ,height)
    seperate_n = 1000
    for i in tqdm(range(0, seperate_n, point_dencity)):
        for j in range(0, seperate_n, point_dencity):
            pTarget = creat_spherial(2000, i * 2 * math.pi/seperate_n, j * math.pi/seperate_n, pSource)
            try:
                pointsIntersection = caster.castRay(pSource, pTarget)
                pcd_list.append(pointsIntersection[0])
                # pcd_list.append(pointsIntersection[1])
                
            except:
                pass

def creat_spherial(r, phi, theta, source_point):

    # phi = phi * 2 * math.pi/100000
    # theta = theta * 2 * math.pi/100000

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


stl_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
	
file_name = "TestSpecimenAssy.stl"
file = os.path.join(stl_folder_path, file_name)

caster = rayCaster.fromSTL(file, scale = 1)
pcd_list = []

point_dencity = 1
axis = [1, 1, 1]
theta = -30

make_pcd_spr2pnt()

pcd_array = np.asarray(pcd_list, dtype=np.float32)

print(pcd_array)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd_array)
# pcd = pcd.voxel_down_sample(voxel_size=0.001)

# o3d.visualization.draw_geometries_with_editing([pcd])
o3d.visualization.draw_geometries([pcd])