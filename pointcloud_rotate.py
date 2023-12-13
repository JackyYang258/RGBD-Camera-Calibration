import numpy as np
import cv2
import open3d as o3d
import numpy as np

camera1_point_cloud = o3d.io.read_point_cloud("/home/hz02/calibration_script/point_cloud/camera.pcd")
transform1 = np.array([[-0.99809669,  0.06025231, -0.01314022,  0.05867366],
 [ 0.01748278 , 0.48079478, 0.87665885, -0.7109162 ],
 [ 0.05913847 , 0.87476056, -0.48093305, 0.48121447],
 [ 0.         , 0.        , 0.         , 1.        ]])
camera1_point_cloud.transform(transform1)

threshold_distance = 1.0

points = np.asarray(camera1_point_cloud.points)
# 计算每个点到点云中心的距离
distances = np.linalg.norm(points, axis=1)

# 保留距离小于阈值的点
filtered_points = points[distances < threshold_distance]

# 创建新的点云对象
filtered_point_cloud = o3d.geometry.PointCloud()
filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

# 创建一个球体的三角网格
mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)

# 将三角网格转换为点云
point_cloud = mesh.sample_points_uniformly(number_of_points=1000)

# 可视化点云
o3d.visualization.draw_geometries([filtered_point_cloud, point_cloud])