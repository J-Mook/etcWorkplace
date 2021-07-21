# import time
# import random
# import sys

# temp = []
# start = time.time()
# for i in range(90000):
#     temp.append(random.normalvariate(0,50))
# sys.stdout = open('/Users/jm/git/etcWorkplace/runtime_check.txt','a')
# print(time.time()-start)

# sys.stdout = open('/Users/jm/git/etcWorkplace/runtime_check.txt','a')
# print("------------------")

# temp = []
# start = time.time()    
# for i in range(90000):
#     temp.append(random.gauss(0,50))
# sys.stdout = open('/Users/jm/git/etcWorkplace/runtime_check.txt','a')
# print(time.time()-start)


# # import the required libraries 
# import random 
# import matplotlib.pyplot as plt 
# from tqdm import tqdm

# # store the random numbers in a list 
# nums = []
# numsg = []
# mu = 0
# sigma = 15000
# abc = [i for i in range(20000)]

# max_j = 11
# # for j in range(1,max_j,2):
# for i in tqdm(range(int(len(abc)/3))): 
#     # temp = random.normalvariate(mu, sigma) 
#     # nums.append(temp)
#     try:
#         # abc_temp = list(filter(lambda x: x <= int(abs(random.gauss(mu, len(abc)/3))), abc))
#         temp1 = abc[int(abs(random.gauss(mu, len(abc)/3)))]

#         temp2 = abc.pop(int(abs(random.gauss(mu, len(abc)/3))))
#         # temp = int(abs(random.gauss(mu, sigma)))
#         # print(temp)
#         nums.append(temp1)
        
#         numsg.append(temp2)
#     except:
#         pass
        
#         # # fig, axes = plt.subplots(1,j)
#         # plt.subplot(max_j,1,j)
#         # plt.hist(nums, bins = 200) 
#         # plt.title(j)
#         # # plt.tight_layout()
# # print(numsg)
# plt.subplot(2,1,1)
# plt.hist(nums, bins = 100) 
# plt.title("append")
# plt.subplot(2,1,2)
# plt.hist(numsg, bins = 100) 
# plt.title("pop")
# plt.tight_layout()
# # plotting a graph 
# # plt.hist(numsg, bins = 200) 
# plt.show()


# import numpy as np

# a = [[2,1],[1,2]]
# b = [[2,1],[2,2]]

# print(np.dot(a, b))

import os, vtk


def castRay(cast,pointRaySource, pointRayTarget):
    # print("adsfsa")

    readerSTL = vtk.vtkSTLReader()  # create a 'vtkSTLReader' object
    readerSTL.SetFileName(stl_file)  # set the .stl filename in the reader
    readerSTL.Update()  # 'update' the reader i.e. read the .stl file
    polydata = readerSTL.GetOutput()

    self_caster = vtk.vtkOBBTree()
    self_caster.SetDataSet(polydata)
    self_caster.BuildLocator()

    try:
        ipts_coords = cast.intersectWithLine(pointRaySource, pointRayTarget)
        print(ipts_coords)
    except:
        return []
    #create a 'vtkPoints' object to store the intersection points
    # pointsVTKintersection = vtk.vtkPoints()

    # #perform ray-casting (intersect a line with the mesh)
    # code = self.caster.IntersectWithLine(pointRaySource,
    #                                      pointRayTarget,
    #                                      pointsVTKintersection, None)
    # if code == 0:
    #     return []
    # elif code == -1:
    #     pass
    return ipts_coords

def fromSTL(filenameSTL, scale=1.0):
    readerSTL = vtk.vtkSTLReader()  # create a 'vtkSTLReader' object
    readerSTL.SetFileName(filenameSTL)  # set the .stl filename in the reader
    readerSTL.Update()  # 'update' the reader i.e. read the .stl file
    polydata = readerSTL.GetOutput()

    # # If there are no points in 'vtkPolyData' something went wrong
    # if polydata.GetNumberOfPoints() == 0:
    #     raise ValueError(
    #         "No point data could be loaded from '" + filenameSTL)
    #     return None

    # Create a new 'rayCaster' with the mesh loaded from the .stl file
    # rT = cls(polydata)

    self_caster = vtk.vtkOBBTree()
    self_caster.SetDataSet(polydata)
    self_caster.BuildLocator()

    #return the 'rayCaster' object
    return self_caster

input_data_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"Lidar_simulator_py", "data")
save_ply_folder_path = os.path.join(input_data_folder_path, "ply")
stl_file_name = "TestSpecimenAssy.stl"
stl_file = os.path.join(input_data_folder_path, stl_file_name)
caster = fromSTL(stl_file, scale = 1)

print(castRay(caster,[0,0,0], [500, 500, 500]))
