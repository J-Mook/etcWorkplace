import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import sklearn

pcd = o3d.io.read_point_cloud("/Users/jm/Desktop/논문/lidar_sample.ply")
picked_pcd = o3d.geometry.PointCloud()
projected_box = o3d.geometry.LineSet()

def calc_projection(a, b):
    # return (np.linalg.norm((np.dot(a, b) / np.dot(b, b)) * b))  # 기존 projection
    return np.dot(a, b) / np.linalg.norm(b)  # 개선된 projection
lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7], [0, 4], [1, 5], [2, 6], [3, 7]]

def most_frequent(data):
    return max(data, key=data.count)

# print(pcd)
# print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])


with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd.cluster_dbscan(eps=50, min_points=50, print_progress=True))
print(labels)
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
picked_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points, dtype=np.float32)[labels == most_frequent(list(labels))])


aabb = picked_pcd.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)
print(aabb.get_center())
print(aabb.get_extent())

temp=[]
x_axis = np.array([1,0,0])
for pnt in aabb.get_box_points():
    dist = calc_projection(pnt, x_axis)
    projected_point = pnt - x_axis * dist
    temp.append(projected_point)

projected_box = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(temp), lines=o3d.utility.Vector2iVector(lines))

o3d.visualization.draw_geometries([pcd, aabb, projected_box])


