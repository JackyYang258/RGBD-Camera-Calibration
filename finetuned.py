'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2023-12-08 18:43:35
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2023-12-11 16:56:42
FilePath: /calibration_script/test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import numpy as np
import cv2
import open3d as o3d
import yaml

STRIDE = 0.005
cam_robot_turple_tuned = {}
with open('./pose/camera_pose_robot.yml', 'r') as f:
    cam_robot_turple = yaml.load(f, Loader=yaml.FullLoader)
    
def custom_draw_geometry_with_key_callback(i):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

    # 将点云添加到窗口中
    vis.add_geometry(point_cloud[i])

    # 初始化变换矩阵
    transform_matrix = np.identity(4)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
    vis.add_geometry(mesh_frame)


    def renew_pcd(transform_matrix):
        pcd_copy = o3d.geometry.PointCloud(point_cloud[i])
        pcd_copy.transform(transform_matrix)
        vis.clear_geometries()
        vis.add_geometry(pcd_copy)
        vis.add_geometry(point_cloud[(i+1)%4])
        vis.add_geometry(point_cloud[(i+2)%4])
        vis.add_geometry(point_cloud[(i+3)%4])
        vis.add_geometry(mesh_frame)
        vis.poll_events()
        vis.update_renderer()

        
    # 定义键盘事件回调函数
    def key_xcallback(vis):
        transform_matrix[0, 3] += STRIDE
        renew_pcd(transform_matrix)

    def key_mxcallback(vis):
        transform_matrix[0, 3] -= STRIDE
        renew_pcd(transform_matrix)
        
    def key_ycallback(vis):
        transform_matrix[1, 3] += STRIDE
        renew_pcd(transform_matrix)
        
    def key_mycallback(vis):
        transform_matrix[1, 3] -= STRIDE
        renew_pcd(transform_matrix)
        
    def key_zcallback(vis):
        transform_matrix[2, 3] += STRIDE
        renew_pcd(transform_matrix)
        
    def key_mzcallback(vis):
        transform_matrix[2, 3] -= STRIDE
        renew_pcd(transform_matrix)

    # 设置键盘事件回调函数
    vis.register_key_callback(ord('A'),key_xcallback)
    vis.register_key_callback(ord('D'),key_mxcallback)
    vis.register_key_callback(ord('W'),key_ycallback)
    vis.register_key_callback(ord('S'),key_mycallback)
    vis.register_key_callback(ord('Z'),key_zcallback)
    vis.register_key_callback(ord('X'),key_mzcallback)
    # 运行可视化
    vis.run()
    T_cam_robot_tuned = np.matmul(transform_matrix, np.array(cam_robot_turple['cam' + str(i+1)]))
    cam_robot_turple_tuned['cam' + str(i+1)] = T_cam_robot_tuned.tolist()
    return transform_matrix


point_cloud1 = o3d.io.read_point_cloud("./point_cloud/pcd1.pcd")
point_cloud2 = o3d.io.read_point_cloud("./point_cloud/pcd2.pcd")
point_cloud3 = o3d.io.read_point_cloud("./point_cloud/pcd3.pcd")
point_cloud4 = o3d.io.read_point_cloud("./point_cloud/pcd4.pcd")
point_cloud = [point_cloud1, point_cloud2, point_cloud3, point_cloud4]

for i in range(4):
    transform_i = custom_draw_geometry_with_key_callback(i)
    point_cloud[i].transform(transform_i)

with open('./pose/camera_pose_robot_tuned.yml', 'w') as f:
    yaml.dump(cam_robot_turple_tuned, f)
