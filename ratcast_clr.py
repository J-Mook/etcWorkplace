# from pycaster.pycaster import rayCaster
import os
import open3d as o3d
import numpy as np
from tqdm import tqdm
import math
import vtk

class rayCaster(object):
    def __init__(self, mesh):
        
        self.mesh = mesh  # set the 'mesh'
        self.caster = None
        self._initCaster()  # create a caster

    @classmethod
    def fromSTL(cls, filenameSTL, scale=1.0):
        
        readerSTL = vtk.vtkSTLReader()  # create a 'vtkSTLReader' object
        readerSTL.SetFileName(
            filenameSTL)  # set the .stl filename in the reader
        readerSTL.Update()  # 'update' the reader i.e. read the .stl file
        polydata = readerSTL.GetOutput()

        if polydata.GetNumberOfPoints() == 0:
            raise ValueError(
                "No point data could be loaded from '" + filenameSTL)
            return None

        rT = cls(polydata)

        if scale != 1.0:
            rT.scaleMesh(scale)
        return rT

    def scaleMesh(self, scale):
        transform = vtk.vtkTransform()
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transform.Scale(scale, scale, scale)  #assuming uniform scale
        transformFilter.SetInput(self.mesh)
        transformFilter.SetTransform(transform)
        transformFilter.Update()
        self.mesh = transformFilter.GetOutput()
        self._updateCaster()

    def _initCaster(self):
        self.caster = vtk.vtkOBBTree()
        self.caster.SetDataSet(self.mesh)
        self.caster.BuildLocator()

    def castRay(self, pointRaySource, pointRayTarget):
        pointsVTKintersection = vtk.vtkPoints()
        code = self.caster.IntersectWithLine(pointRaySource, pointRayTarget, pointsVTKintersection, None)

        if code == 0:
            return []
        elif code == -1:
            pass
        pointsVTKIntersectionData = pointsVTKintersection.GetData()
        noPointsVTKIntersection = pointsVTKIntersectionData.GetNumberOfTuples()
        pointsIntersection = []
        for idx in range(noPointsVTKIntersection):
            _tup = pointsVTKIntersectionData.GetTuple3(idx)
            pointsIntersection.append(_tup)

        return pointsIntersection


def make_pcd_spr2pnt(pSource, seperate_n):
    # point to surface
    for i in tqdm(range(0, seperate_n)):
        for j in range(0, seperate_n):
            pTarget = creat_spherial(3000, i * 2 * math.pi/seperate_n, j * math.pi/seperate_n, pSource)
            try:
                pointsIntersection = caster.castRay(pSource, pTarget)
                pcd_list.append(pointsIntersection[0])
                
            except:
                pass

def creat_spherial(r, phi, theta, source_point):

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

def find_sphere_redius(file):
    mesh = o3d.io.read_triangle_mesh(file)
    mesh = mesh.sample_points_poisson_disk(1000)
    o3d.visualization.draw_geometries([mesh])
    mesh = np.array(mesh.points)
        
    longest_distance = 0
    for point in tqdm(mesh):
        longest_distance = max(math.dist(point,Source_point),longest_distance)

    return longest_distance


pcd_list = []
pcd = o3d.geometry.PointCloud()
    
stl_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
file_name = "TestSpecimenAssy.stl"
stl_file = os.path.join(stl_folder_path, file_name)
caster = rayCaster.fromSTL(stl_file, scale = 1)

Source_point = [800, -1000.0, 500] #in sample data coordinate (width ,depth ,height)
seperate_spr = 100 #
redius = find_sphere_redius(stl_file)
make_pcd_spr2pnt(Source_point, seperate_spr, redius)

pcd_array = np.asarray(pcd_list, dtype=np.float32)
# print(pcd_array)
pcd.points = o3d.utility.Vector3dVector(pcd_array)
# pcd = pcd.voxel_down_sample(voxel_size=0.001)

o3d.visualization.draw_geometries([pcd]) #sanning data check