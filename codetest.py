# from pycaster.pycaster import rayCaster
# import inspect
# print(inspect.getfile(os))




# # import time
# # import random
# # import sys

# # temp = []
# # start = time.time()
# # for i in range(90000):
# #     temp.append(random.normalvariate(0,50))
# # sys.stdout = open('/Users/jm/git/etcWorkplace/runtime_check.txt','a')
# # print(time.time()-start)

# # sys.stdout = open('/Users/jm/git/etcWorkplace/runtime_check.txt','a')
# # print("------------------")

# # temp = []
# # start = time.time()    
# # for i in range(90000):
# #     temp.append(random.gauss(0,50))
# # sys.stdout = open('/Users/jm/git/etcWorkplace/runtime_check.txt','a')
# # print(time.time()-start)


# # # import the required libraries 
# # import random 
# # import matplotlib.pyplot as plt 
# # from tqdm import tqdm

# # # store the random numbers in a list 
# # nums = []
# # numsg = []
# # mu = 0
# # sigma = 15000
# # abc = [i for i in range(20000)]

# # max_j = 11
# # # for j in range(1,max_j,2):
# # for i in tqdm(range(int(len(abc)/3))): 
# #     # temp = random.normalvariate(mu, sigma) 
# #     # nums.append(temp)
# #     try:
# #         # abc_temp = list(filter(lambda x: x <= int(abs(random.gauss(mu, len(abc)/3))), abc))
# #         temp1 = abc[int(abs(random.gauss(mu, len(abc)/3)))]

# #         temp2 = abc.pop(int(abs(random.gauss(mu, len(abc)/3))))
# #         # temp = int(abs(random.gauss(mu, sigma)))
# #         # print(temp)
# #         nums.append(temp1)
        
# #         numsg.append(temp2)
# #     except:
# #         pass
        
# #         # # fig, axes = plt.subplots(1,j)
# #         # plt.subplot(max_j,1,j)
# #         # plt.hist(nums, bins = 200) 
# #         # plt.title(j)
# #         # # plt.tight_layout()
# # # print(numsg)
# # plt.subplot(2,1,1)
# # plt.hist(nums, bins = 100) 
# # plt.title("append")
# # plt.subplot(2,1,2)
# # plt.hist(numsg, bins = 100) 
# # plt.title("pop")
# # plt.tight_layout()
# # # plotting a graph 
# # # plt.hist(numsg, bins = 200) 
# # plt.show()


# # import numpy as np

# # a = [[2,1],[1,2]]
# # b = [[2,1],[2,2]]

# # print(np.dot(a, b))

# # import os, vtk


# # def castRay(cast,pointRaySource, pointRayTarget):
# #     # print("adsfsa")

# #     readerSTL = vtk.vtkSTLReader()  # create a 'vtkSTLReader' object
# #     readerSTL.SetFileName(stl_file)  # set the .stl filename in the reader
# #     readerSTL.Update()  # 'update' the reader i.e. read the .stl file
# #     polydata = readerSTL.GetOutput()

# #     self_caster = vtk.vtkOBBTree()
# #     self_caster.SetDataSet(polydata)
# #     self_caster.BuildLocator()

# #     try:
# #         ipts_coords = cast.intersectWithLine(pointRaySource, pointRayTarget)
# #         print(ipts_coords)
# #     except:
# #         return []
# #     #create a 'vtkPoints' object to store the intersection points
# #     # pointsVTKintersection = vtk.vtkPoints()

# #     # #perform ray-casting (intersect a line with the mesh)
# #     # code = self.caster.IntersectWithLine(pointRaySource,
# #     #                                      pointRayTarget,
# #     #                                      pointsVTKintersection, None)
# #     # if code == 0:
# #     #     return []
# #     # elif code == -1:
# #     #     pass
# #     return ipts_coords

# # def fromSTL(filenameSTL, scale=1.0):
# #     readerSTL = vtk.vtkSTLReader()  # create a 'vtkSTLReader' object
# #     readerSTL.SetFileName(filenameSTL)  # set the .stl filename in the reader
# #     readerSTL.Update()  # 'update' the reader i.e. read the .stl file
# #     polydata = readerSTL.GetOutput()

# #     # # If there are no points in 'vtkPolyData' something went wrong
# #     # if polydata.GetNumberOfPoints() == 0:
# #     #     raise ValueError(
# #     #         "No point data could be loaded from '" + filenameSTL)
# #     #     return None

# #     # Create a new 'rayCaster' with the mesh loaded from the .stl file
# #     # rT = cls(polydata)

# #     self_caster = vtk.vtkOBBTree()
# #     self_caster.SetDataSet(polydata)
# #     self_caster.BuildLocator()

# #     #return the 'rayCaster' object
# #     return self_caster

# # input_data_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"Lidar_simulator_py", "data")
# # save_ply_folder_path = os.path.join(input_data_folder_path, "ply")
# # stl_file_name = "TestSpecimenAssy.stl"
# # stl_file = os.path.join(input_data_folder_path, stl_file_name)
# # caster = fromSTL(stl_file, scale = 1)

# # print(castRay(caster,[0,0,0], [500, 500, 500]))

# # import open3d as o3d
# # import os
# # import numpy as np

# # pcdpath = os.path.join(os.path.realpath(__file__), "data", "TestSpecimenAssy.stl")
# # pcd = o3d.io.read_point_cloud(r"/Users/jm/git/etcWorkplace/data/airplane_0627_poisson_sampling_37797pts.ply")
# # msh = o3d.io.read_triangle_mesh(r"/Users/jm/git/etcWorkplace/data/airplane_0627.stl")
# # print(pcd)#Output the number of point cloud points
# # box = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100.0, origin=np.array([50.0, 50.0, 50.0]))
# # o3d.visualization.draw_geometries([msh, box, pcd])

# # aabb = pcd.get_axis_aligned_bounding_box()
# # aabb.color = (1,0,0)#aabb bounding box is red
# # obb = pcd.get_oriented_bounding_box()
# # obb.color = (0,1,0)#obbBounding box is green
# # o3d.visualization.draw_geometries([pcd, aabb, obb])

# import vtk
# import open3d as o3d
# import time

# # # mesh = o3d.io.read_triangle_mesh(r"/Users/jm/git/etcWorkplace/data/airplane_0627.stl")

# # def GetIntersect(obbTree, pSource, pTarget):
# #     global start
# #     global end

# #     points = vtk.vtkPoints()
# #     cellIds = vtk.vtkIdList()

# #     # Perform intersection test
# #     code = obbTree.IntersectWithLine(pSource, pTarget, points, cellIds)
    
# #     pointData = points.GetData()
# #     # noPoints = pointData.GetNumberOfTuples()
# #     # noIds = cellIds.GetNumberOfIds()

# #     pointsInter = []
# #     # # cellIdsInter = []
# #     # for idx in range(noPoints):
# #     #     print(idx)
# #     #     pointsInter.append(pointData.GetTuple3(idx))
# #     #     # cellIdsInter.append(cellIds.GetId(idx))

# #     # pointsInter.append(pointData.GetTuple3(0))
# #     pointsInter.append(obbTree.IntersectWithLine())
# #     return pointsInter

# reader = vtk.vtkSTLReader()
# reader.SetFileName(r"/Users/jm/git/etcWorkplace/data/TestSpecimenAssy.stl")
# reader.Update()
# readed = reader.GetOutput()

# mesh = vtk.vtkPolygon()
# mesh.SetDataSet(readed)
# mesh.Update()

# # obbTree = vtk.vtkOBBTree()
# # obbTree.SetDataSet(mesh)
# # obbTree.BuildLocator()

# # # code = obbTree.IntersectWithLine((100.0, 100.0, 0.0), (0.0, 0.0, 0.0), points, cellIds)
# # for i in range(37797):
# #     inter = GetIntersect(obbTree, (100.0, 100.0, 0.0), (0.0, 0.0, 0.0))


# t = vtk.mutable(0)  # Parametric coordinate of intersection (0 (corresponding to p1) to 1 (corresponding to p2))
# x = [0.0, 0.0, 0.0]
# pcoords = [0.0, 0.0, 0.0]
# subId = vtk.mutable(0)

# start = time.time()
# iD = mesh.IntersectWithLine((100.0, 100.0, 0.0), (0.0, 0.0, 0.0), 0, t, x, pcoords, subId)

# print('intersected? ', 'Yes' if iD == 1 else 'No')
# print('intersection: ', x)
# end = time.time()

# print(end - start)
# # print(inter[0])

# # from __future__ import print_function

# # import vtk

# # def main():
# #     # Create a square in the x-y plane.
# #     points = vtk.vtkPoints()
# #     points.InsertNextPoint(0.0, 0.0, 0.0)
# #     points.InsertNextPoint(1.0, 0.0, 0.0)
# #     points.InsertNextPoint(1.0, 1.0, 0.0)
# #     points.InsertNextPoint(0.0, 1.0, 0.0)

# #     # Create the polygon
# #     polygon = vtk.vtkPolygon()
# #     polygon.GetPoints().DeepCopy(points)
# #     polygon.GetPointIds().SetNumberOfIds(4)  # The 4 corners of the square
# #     for i in range(4):
# #         polygon.GetPointIds().SetId(i, i)

# #     # Inputs
# #     p1 = [0.1, 0, -1.0]
# #     p2 = [0.1, 0, 1.0]
# #     tolerance = 0.001

# #     # Outputs
# #     t = vtk.mutable(0)  # Parametric coordinate of intersection (0 (corresponding to p1) to 1 (corresponding to p2))
# #     x = [0.0, 0.0, 0.0]
# #     pcoords = [0.0, 0.0, 0.0]
# #     subId = vtk.mutable(0)
# #     iD = polygon.IntersectWithLine(p1, p2, tolerance, t, x, pcoords, subId)

# #     print('intersected? ', 'Yes' if iD == 1 else 'No')
# #     print('intersection: ', x)


# # if __name__ == '__main__':
# # #     main()


# # import open3d as o3d
# import open3d.cpu.pybind as ocpu
# # cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
# # cube1 = o3d.geometry.TriangleMesh.scale(cube ,scale = 1, center = [0,0,0])
# # cube1 = o3d.geometry.TriangleMesh.transform(cube1, transformation = [100,0,0])
# # cube2 = o3d.geometry.TriangleMesh.scale(cube ,scale = 2, center = [0,0,0])
# # cube1 = o3d.geometry.TriangleMesh.transform(cube2, transformation = [200,0,0])
# # cube3 = o3d.geometry.TriangleMesh.scale(cube ,scale = 3, center = [0,0,0])
# # cube1 = o3d.geometry.TriangleMesh.transform(cube3, transformation = [300,0,0])
# # cube4 = o3d.geometry.TriangleMesh.scale(cube ,scale = 4, center = [0,0,0])
# # cube1 = o3d.geometry.TriangleMesh.transform(cube4, transformation = [400,0,0])
# # cube5 = o3d.geometry.TriangleMesh.scale(cube ,scale = 5, center = [0,0,0])
# # cube1 = o3d.geometry.TriangleMesh.transform(cube5, transformation = [500,0,0])



# # cube = o3d.t.geometry.TriangleMesh.from_legacy_triangle_mesh(cube)

# # vi_cube = o3d.io.read_triangle_mesh(cube)
# # # o3d.visualization.draw_geometries([cube1, cube2, cube3, cube4, cube5])

# # scene = ocpu.t.geometry.RaycastingScene()
# # cube_id = scene.add_triangles(cube)

# # # print(cube_id)

# # # rays = o3d.core.Tensor([[0.5, 0.5, 10, 0, 0, -1], [-1, -1, -1, 0, 0, -1]],
# # #                        dtype=o3d.core.Dtype.Float32)

# # # ans = scene.cast_rays(rays)

# # # print(ans.keys())


# # # print(ans['t_hit'].numpy(), ans['geometry_ids'].numpy())


# # # cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
# # # cube = o3d.t.geometry.TriangleMesh.from_legacy_triangle_mesh(cube)
# # torus = o3d.geometry.TriangleMesh.create_torus().translate([0, 0, 2])
# torus = o3d.t.geometry.TriangleMesh.from_legacy_triangle_mesh(torus)
# sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5).translate(
#     [1, 2, 3])
# sphere = o3d.t.geometry.TriangleMesh.from_legacy_triangle_mesh(sphere)

# scene = o3d.t.geometry.RaycastingScene()
# scene.add_triangles(cube)
# scene.add_triangles(torus)
# _ = scene.add_triangles(sphere)

# rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
#     fov_deg=90,
#     center=[0, 0, 2],
#     eye=[2, 3, 0],
#     up=[0, 1, 0],
#     width_px=640,
#     height_px=480,
# )
# # Ee can directly pass the rays tensor to the cast_rays function.
# ans = scene.cast_rays(rays)

# import trimesh
# import open3d as o3d
# import numpy as np
# import os

# input_data_folder_path = str(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data"))
# stl_file_name = "TestSpecimenAssy.stl"
# # stl_file_name = "MeshCAA.stl"
# # stl_file_name = "airplane_0627.stl"

# stl_file = os.path.join(input_data_folder_path, stl_file_name)

# mesh_read = trimesh.load_mesh(stl_file)

# # mesh = trimesh.ray.ray_triangle.RayMeshIntersector(mesh_read)
# pcd = o3d.geometry.PointCloud()

# ray_origins = np.array([[0, 0, -3], [2, 2, -3]])
# ray_directions = np.array([[0, 0, 1], [0, 0, 1]])

# # ray_origins = np.array([0, 0, -3])
# # ray_directions = np.array([0, 0, 1])

# pcd_list, _, _ = mesh_read.ray.intersects_location(ray_origins=ray_origins, ray_directions=ray_directions)

# pcd_array = np.asarray(pcd_list, dtype=np.float32)
# pcd.points = o3d.utility.Vector3dVector(pcd_array)

# print(pcd)

# o3d.visualization.draw_geometries([pcd])


import numpy as np
import random
import time
test_num = 1000000
list1 = np.array([[1, 1, 1] for i in range(0,test_num)])

time1 = time.time()####################################################
listadd = np.array([[random.random(), random.random(), random.random()] for j in range(0,test_num)])
list1_out = np.array([[]])
# print(listadd)
list1_out = np.add(list1,listadd)
time1 = time.time() - time1####################################################

list2 = [[1,1,1] for i in range(test_num)]
list2_out = [[0, 0, 0] for i in range(test_num)]

time2 = time.time()####################################################
listadd = np.array([[random.random(), random.random(), random.random()] for j in range(0,test_num)])

for i in range(len(list2)):
    # list2_out[i][0] = list2[i][0] + random.random()
    # list2_out[i][1] = list2[i][1] + random.random()
    # list2_out[i][2] = list2[i][2] + random.random()
    
    list2_out[i][0] = list2[i][0] + listadd[0]
    list2_out[i][1] = list2[i][1] + listadd[1]
    list2_out[i][2] = list2[i][2] + listadd[2]
    
time2 = time.time() - time2####################################################

print("----------------------")
print("test num :",test_num)
print(time1)
print(time2)
print("effecient :",(time2 - time1)*100/time2," %")
print("----------------------")
