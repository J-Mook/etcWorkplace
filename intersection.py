#!/bin/python
from __future__ import print_function
import open3d as o3d
import numpy as np
import pandas as pd
import os
from tqdm import tqdm

def creat_mesh():

	ply_folder_path = os.path.dirname(os.path.realpath(__file__))
	# ply_name = 'temp.png'

	ply_list = sorted(os.listdir(ply_folder_path + '/data'))
	
	pcd_test = o3d.geometry.PointCloud()
	# for ply in ply_list:
	# 	pcd_load = o3d.io.read_point_cloud(os.path.join(ply_folder_path,ply))

	pcd_load = o3d.io.read_point_cloud(os.path.join(ply_folder_path,ply_list[0]))
	pcd_load = pcd_load.voxel_down_sample(voxel_size=0.001)
	print(type(pcd_load))
	print(pcd_load)
	o3d.visualization.draw_geometries([pcd_load])

def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
 
	ndotu = planeNormal.dot(rayDirection)
	if abs(ndotu) < epsilon:
		raise RuntimeError("no intersection or line is within plane")
 
	w = rayPoint - planePoint
	si = -planeNormal.dot(w) / ndotu
	Psi = w + si * rayDirection + planePoint
	return Psi


if __name__=="__main__":
	#Define plane
	# planeNormal = np.array([0, 0, 2])
	# planePoint = np.array([0, 0, 5]) #Any point on the plane
 
	# #Define ray
	# rayDirection = np.array([0, -1, 1])
	# rayPoint = np.array([0, 0, 10]) #Any point along the ray
	
	# Psi = LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint)
	# print ("intersection at", Psi)

	creat_mesh()