#generate a point cloud from a certain camera 

import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import copy
import time
from helpers import *
import yaml

VISUALIZE = True
DEVICE1 = '213522070411'
DEVICE2 = '241122073122'
DEVICE3 = '215222077044'
DEVICE4 = '215222074858'
TUNED = True
if TUNED:
    CAMERA_POSE = './pose/camera_pose_robot_tuned.yml'
else:
    CAMERA_POSE = './pose/camera_pose_robot.yml'

def get_point_cloud(pipeline, extrinsic_transform):
    # Wait for a coherent pair of frames: depth and color
    for _ in range(10):
        frames = pipeline.wait_for_frames()

    frames = pipeline.wait_for_frames()
    
    align_to = rs.stream.depth
    align = rs.align(align_to)

    aligned_frames = align.process(frames)
    
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    profile = frames.get_profile()

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    #480*640
    color_image = np.asanyarray(color_frame.get_data())

    print("type of depth_image:",type(depth_image))
    print("shape of depth_image:",depth_image.shape)

    o3d_color = o3d.geometry.Image(color_image)
    o3d_depth = o3d.geometry.Image(depth_image)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth,
                                                                    depth_scale=1000.0,
                                                                    depth_trunc=3.0,
                                                                    convert_rgb_to_intensity=False)

    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    # 转换为open3d中的相机参数
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width, intrinsics.height,
        intrinsics.fx, intrinsics.fy,
        intrinsics.ppx, intrinsics.ppy
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            pinhole_camera_intrinsic,
            # np.linalg.inv(extrinsic_transform)
        )
    
    pcd.transform(extrinsic_transform)
    
    return pcd


def main():
    pipeline1 = rs.pipeline()
    pipeline2 = rs.pipeline()
    pipeline3 = rs.pipeline()
    pipeline4 = rs.pipeline()

    config1 = rs.config()
    config1.enable_device(DEVICE1)
    config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    config2 = rs.config()
    config2.enable_device(DEVICE2)
    config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config2.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    config3 = rs.config()
    config3.enable_device(DEVICE3)
    config3.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config3.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    config4 = rs.config()
    config4.enable_device(DEVICE4)
    config4.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config4.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    pipeline1.start(config1)
    pipeline2.start(config2)
    pipeline3.start(config3)
    pipeline4.start(config4)

    with open(CAMERA_POSE, 'r') as f:
        transform = yaml.load(f, Loader=yaml.FullLoader)
    
    transform1 = np.array(transform['cam1'])
    transform2 = np.array(transform['cam2'])
    transform3 = np.array(transform['cam3'])
    transform4 = np.array(transform['cam4'])

    pcd1 = get_point_cloud(pipeline1, transform1)
    pcd2 = get_point_cloud(pipeline2, transform2)
    pcd3 = get_point_cloud(pipeline3, transform3)
    pcd4 = get_point_cloud(pipeline4, transform4)

    o3d.io.write_point_cloud("./point_cloud/pcd1.pcd", pcd1)
    o3d.io.write_point_cloud("./point_cloud/pcd2.pcd", pcd2)
    o3d.io.write_point_cloud("./point_cloud/pcd3.pcd", pcd3)
    o3d.io.write_point_cloud("./point_cloud/pcd4.pcd", pcd4)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])

    # print(np.asarray(pcd1.points).shape[0])
    # o3d.visualization.draw_geometries([pcd1, pcd2, pcd3, mesh_frame])

    n_points = 6000
    min_bound = np.array([0.1, -0.35, -0.1])
    max_bound = np.array([1, 0.35, 0.6])
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    merged_pcd = pcd1.crop(bbox) + pcd2.crop(bbox) + pcd3.crop(bbox) + pcd4.crop(bbox)
    sample_ratio = n_points / np.asarray(merged_pcd.points).shape[0]
    # o3d.visualization.draw_geometries([merged_pcd.random_down_sample(sample_ratio),
    #                                    mesh_frame])
    o3d.visualization.draw_geometries([merged_pcd.farthest_point_down_sample(n_points),
                                       mesh_frame])

main()