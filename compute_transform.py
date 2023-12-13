'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2023-12-04 16:56:57
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2023-12-04 18:00:41
FilePath: /calibration_script/compute_transform.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import numpy as np
import os
import sys
import yaml
from transforms3d.quaternions import *

def ar_tag_transform():
    with open('./pose/robot_state.yml', 'r') as f:
        robot_state = yaml.load(f, Loader=yaml.FullLoader)
    fingertip_pos = np.array(robot_state['fingertip_pos'])
    ar_tag_rot = quat2mat(axangle2quat([0,0,1],np.pi/2))
    T_tag_robot = np.concatenate((np.concatenate((ar_tag_rot, fingertip_pos.reshape(3,1)), axis=1), [[0, 0, 0, 1]]))
    
    cam_robot_turple = {}
    for i in range(1,5):
        with open('./pose/cam' + str(i) + '_tag.yml', 'r') as f:
            camera_tag_dict = yaml.load(f, Loader=yaml.FullLoader)
            
        # transformation of tag in the camera frame
        camera_tag_orientation = np.array(list(camera_tag_dict['orientation'].values()))
        camera_tag_orientation = np.roll(camera_tag_orientation,1)
        cam_rot = quat2mat(camera_tag_orientation)
        camera_tag_position = np.array(list(camera_tag_dict['position'].values())).reshape(3,1)
        T_cam_tag = np.concatenate((np.concatenate((cam_rot, camera_tag_position), axis=1), [[0, 0, 0, 1]]))
        T_cam_tag = np.linalg.inv(T_cam_tag)
        T_cam_robot = np.matmul(T_tag_robot, T_cam_tag)
        cam_robot_turple['cam' + str(i)] = T_cam_robot.tolist()

    with open('./pose/camera_pose_robot.yml', 'w') as f:
        yaml.dump(cam_robot_turple, f)
    print(cam_robot_turple)
    
ar_tag_transform()