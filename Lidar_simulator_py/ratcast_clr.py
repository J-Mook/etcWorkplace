# from numpy.core.records import array
# from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math, vtk
from plyfile import PlyElement, PlyData
PI = 3.14159265358979

##initialize
Source_point = [500, 500, 400] #lidar location (width ,depth ,height)
Resolution = 3200 #liar point amount
camera_moving_mount = 1
tSource = [0,0, 200]
select_model = 'm'

medels_data = {'s' : [[343, 360, 382],[384,442,520]], 'm' : [[317, 590, 826],[458, 650, 1118]], 'l' : [[600, 1082, 1644],[870, 1239, 2150]]} #[[width],[lenth]]
seperate_spr = int(math.sqrt(Resolution))

def fromSTL(filenameSTL):

        readerSTL = vtk.vtkSTLReader()  # create a 'vtkSTLReader' object
        readerSTL.SetFileName(filenameSTL)  # set the .stl filename in the reader
        readerSTL.Update()  # 'update' the reader i.e. read the .stl file
        polydata = readerSTL.GetOutput()

        # If there are no points in 'vtkPolyData' something went wrong
        if polydata.GetNumberOfPoints() == 0:
            raise ValueError(
                "No point data could be loaded from '" + filenameSTL)
            return None
        # Create a new 'rayCaster' with the mesh loaded from the .stl file
        # rT = cls(polydata)
        #if a non-unit 'scale' is given then scale the polydata
        return polydata


def castRay(cast, pointRaySource, pointRayTarget):
        #create a 'vtkOBBTree' object
        caster = vtk.vtkOBBTree()
        #set the 'mesh' as the caster's dataset
        caster.SetDataSet(cast.mesh)
        #build a caster locator
        caster.BuildLocator()
        #create a 'vtkPoints' object to store the intersection points
        pointsVTKintersection = vtk.vtkPoints()

        #perform ray-casting (intersect a line with the mesh)
        code = cast.IntersectWithLine(pointRaySource, pointRayTarget, pointsVTKintersection, None)

        if code == 0:
            return []
        elif code == -1:
            pass
        #get the actual data of the intersection points (the point tuples)
        pointsVTKIntersectionData = pointsVTKintersection.GetData()
        #get the number of tuples
        noPointsVTKIntersection = pointsVTKIntersectionData.GetNumberOfTuples()

        #create an empty list that will contain all list objects
        pointsIntersection = []
        print(noPointsVTKIntersection)
        print("sdasdadasdasd")
        idx = noPointsVTKIntersection[0]
        pointsIntersection.append(pointsVTKIntersectionData.GetTuple3(idx))
        
        #return the list of list objects
        return pointsIntersection


def make_pcd_spr2pnt(pSource, seperate_n, sphere_redius):
    # point to surface
    pcd = []
    min_angle_xy, max_anlge_xy, min_angle_z, max_anlge_z = setting_ROI_angle(pSource, tSource)
    
    for i in tqdm(range(0, seperate_n), leave = False, position = 2):
        for j in range(0, seperate_n):
            pTarget = creat_sphere(sphere_redius, (i * (max_anlge_z - min_angle_z) / seperate_n) + min_angle_z, (j * (max_anlge_xy - min_angle_xy) / seperate_n) + min_angle_xy, pSource)
            try:
                pointsIntersection = castRay(caster, pSource, pTarget)
                if check_fov(pSource, pointsIntersection[0]):
                    pcd.append(pointsIntersection[0])
            except:
                pass
    return pcd
    
def setting_ROI_angle(start_point, end_point):
    source_angle_xy = math.atan2(0 - start_point[1], 0 - start_point[0])
    # source_angle_z = cal_angle([start_point[0], start_point[1], 0], [ 0 - start_point[0], 0 - start_point[1], start_point[2] + target_z])
    source_angle_z = cal_angle([start_point[0],start_point[1], 0], [start_point[0] - end_point[0],
                                 start_point[1] - end_point[1], start_point[2] - end_point[2]])
    
    # xy_min = source_angle_xy - PI/8
    # xy_max = source_angle_xy + PI/8
    # z_min = source_angle_z - PI/8 + PI/2
    # z_max = source_angle_z + PI/8 + PI/2
    xy_min = source_angle_xy - PI/8
    xy_max = source_angle_xy + PI/8
    z_min = source_angle_z - math.radians(17) + PI/2
    z_max = source_angle_z + math.radians(17) + PI/2
    
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
    if math.dist(source_point, check_point) > medels_data[select_model][1][0]:
        return True
    else:
        return False

##search stl file data
stl_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
save_ply_folder_path = os.path.join(stl_folder_path, "ply")

# stl_list = sorted(os.listdir(stl_folder_path))
# for stl_file_name in stl_list:
stl_file_name = "TestSpecimenAssy.stl"
stl_file = os.path.join(stl_folder_path, stl_file_name)
caster = fromSTL(stl_file)

runtime=[]
##scanning multiple location
for move_num in tqdm(range(camera_moving_mount),leave = False, position = 0):
    
    pcd_list = []
    pcd = o3d.geometry.PointCloud()

    if camera_moving_mount != 1:
        now_point = camera_move(Source_point, move_num)
        # redius = find_sphere_redius(stl_file, now_point)
        redius = medels_data[select_model][1][2]
        pcd_list = make_pcd_spr2pnt(now_point, seperate_spr, redius)
    else:
        # redius = find_sphere_redius(stl_file, Source_point)
        redius = medels_data[select_model][1][2]
        pcd_list = make_pcd_spr2pnt(Source_point, seperate_spr, redius)

    ## scanning data convert to pointcloud data
    pcd_array = np.asarray(pcd_list, dtype=np.float32)
    pcd.points = o3d.utility.Vector3dVector(pcd_array)
    # pcd = pcd.voxel_down_sample(voxel_size=0.001)

    ##scanning data visulaization
    # print(len(np.array(pcd.points)))
    if pcd_array.size != 0:
        # o3d.visualization.draw_geometries([pcd])
        ply_name = stl_file_name.split(".")[0]+ "_" +format(seperate_spr, '04') + ".ply"
        save_path = os.path.join(save_ply_folder_path, ply_name)
        save_float_ply(pcd_array, save_path)
        pass
    else:
        print("!!!pointcloud is empty!!!")

o3d.visualization.draw_geometries([pcd])
