from numpy.lib.function_base import average
import open3d as o3d
import numpy as np
import os

ply_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"data","ply")
ply_list = sorted(os.listdir(ply_folder_path))
arr_avg = []
for ply in ply_list:
    # print(ply)
    pcd_load = o3d.io.read_point_cloud(os.path.join(ply_folder_path, ply))
    arr = np.array(pcd_load.points)
    if pcd_load != []:
        print(len(arr))
        arr_avg.append(len(arr))
        # o3d.visualization.draw_geometries_with_editing([pcd_load])
    else:
        print("this file is empty!!")

print("average mount of pointcloud", average(arr_avg))
# pcd_load = pcd_load.voxel_down_sample(voxel_size=0.1)
