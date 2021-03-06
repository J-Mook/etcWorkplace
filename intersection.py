#!/bin/python
from __future__ import print_function
import open3d as o3d
import numpy as np
import pandas as pd
import os
from tqdm import tqdm

def creat_mesh():
	
	ply_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data')
	
	ply_list = sorted(os.listdir(ply_folder_path))
	
	# pcd_load = o3d.io.read_point_cloud(os.path.join(ply_folder_path, ply_list[0]))
	# pcd_load = pcd_load.voxel_down_sample(voxel_size=0.001)
	
	ply_mesh = o3d.io.read_triangle_mesh(os.path.join(ply_folder_path, ply_list[0]))
	ply_mesh.compute_vertex_normals()

	ply_mesh_tri = np.asarray(ply_mesh.triangles)
	ply_mesh_vert = np.asarray(ply_mesh.vertices)
	ply_mesh_norm = np.asarray(ply_mesh.triangle_normals)
	
	for i, triangle in enumerate(ply_mesh_tri[0:3]):
		a = LinePlaneCollision(ply_mesh_norm[i],ply_mesh_vert[i][0], rayDirection, rayPoint, epsilon=1e-6)
		
		print(a)
		# print(ply_mesh_vert[ply_mesh_tri[i][1]])
		# print(ply_mesh_vert[ply_mesh_tri[i][2]])
		
	# for ply_meshes in ply_mesh_arr:	
		# print(ply_meshes)
		
	# o3d.visualization.draw_geometries([ply_mesh])

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
 
	#Define ray
	rayDirection = np.array([0, -1, -1])
	rayPoint = np.array([0, 0, 0]) #Any point along the ray
	
	# Psi = LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint)
	# print ("intersection at", Psi)

	creat_mesh()