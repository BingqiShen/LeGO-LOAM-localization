import open3d as o3d
import numpy as np
import math


# 1. read pointcloud & pose
raw_path = '/home/thinking/Downloads/lego-LOAM/cloudGlobal.pcd'
raws_pcd = o3d.io.read_point_cloud(raw_path)
raw_points = np.array(raws_pcd.points)
print(raw_points)

revert_points = raw_points.copy()
revert_points[:, 0] = raw_points[:, 2]
revert_points[:, 1] = raw_points[:, 0]
revert_points[:, 2] = raw_points[:, 1]

revert_pcd = o3d.geometry.PointCloud()
revert_pcd.points = o3d.utility.Vector3dVector(revert_points)

print(revert_points)
revert_path = '/home/thinking/Downloads/lego-LOAM/cloudGlobal_revert.pcd'
o3d.io.write_point_cloud(revert_path, revert_pcd)

axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=[0, 0, 0])
# o3d.visualization.draw_geometries([revert_pcd, axis_pcd])