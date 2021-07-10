import pycaster # it should run on python2 environmonet
import os
import open3d as o3d
from pycaster.test import test_all

ply_folder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data')
	
# ply_list = sorted(os.listdir(ply_folder_path))
file_name = "TestSpecimenAssy.stl"
file = os.path.join(ply_folder_path, file_name)

ply_mesh = o3d.io.read_triangle_mesh(os.path.join(file))

caster = pycaster.rayCaster.fromSTL(file)

pSource = [100.0, 100.0, 0.0]
pTarget = [0.0, 0.0, 0.0]
pointsIntersection = caster.castRay(pSource, pTarget)

test_all.runTests()
