import open3d as o3d
import numpy as np
import pandas as pd
import os
from tqdm import tqdm

ply_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"data","ply")
# ply_folder_path = r"C:\Users\pc\Desktop\Data\ply"
ply_list = sorted(os.listdir(ply_folder_path))
print(ply_list)

# pcd_test = o3d.geometry.PointCloud()

# for i in tqdm(range(len(ply_list))):
#     pcd_load = o3d.io.read_point_cloud(os.path.join(ply_folder_path,ply_list[i]))
#     pcd_load = pcd_load.voxel_down_sample(voxel_size=0.001)
#     # pcd_print.append(pcd_load)
#     pcd_test += pcd_load
# arr = np.array(pcd_load.points)

pcd_load = o3d.io.read_point_cloud(ply_list[0])

pcd_load = pcd_load.voxel_down_sample(voxel_size=0.1)
# print(type(pcd_test))
arr = np.array(pcd_load.points)


# data_xyz = pd.DataFrame(arr, columns =["x", "y", "z"])

# print(data_xyz)

o3d.visualization.draw_geometries_with_editing([arr])
