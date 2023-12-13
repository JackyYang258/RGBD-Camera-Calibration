'''
Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Date: 2023-12-04 16:56:57
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2023-12-11 19:56:43
FilePath: /calibration_script/robot_state.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import numpy as np
import os
import sys
import yaml

from polymetis import RobotInterface
from transforms3d.quaternions import *

def get_robot_state():
    robot = RobotInterface(
        ip_address="172.16.0.8",
        enforce_version=False,
    )
    
    robot_state = {}
    ee_pos, ee_quat = robot.pose_ee()
    ee_ori = np.array([ee_quat[3], ee_quat[0], ee_quat[1], ee_quat[2]])
    print(f"Current ee position: {ee_pos}")
    print(f"Current ee orientation: {ee_ori} (wxyz)")
    robot_state['ee_pos'] = ee_pos.tolist()
    robot_state['ee_ori'] = ee_ori.tolist()

    ee_fingertip_T_mat = np.array([[0.707, 0.707, 0, 0], [-0.707, 0.707, 0, 0], [0, 0, 1, 0.1124], [0, 0, 0, 1]])
    
    fingertip_pos = (quat2mat(ee_ori) @ ee_fingertip_T_mat[:3, 3].T).T + ee_pos.numpy()
    fingertip_ori = qmult(ee_ori, mat2quat(ee_fingertip_T_mat[:3, :3]))
    
    print(f"Current fingertip position: {fingertip_pos}")
    print(f"Current fingertip orientation: {fingertip_ori}  (wxyz)")
    
    robot_state['fingertip_pos'] = fingertip_pos.tolist()
    robot_state['fingertip_ori'] = fingertip_ori.tolist()
    
    # with open('./pose/robot_state.yml', 'w') as f:
    #     yaml.dump(robot_state, f)
    print(robot_state)


get_robot_state()

#{'ee_pos': [0.6494057774543762, 0.007566374726593494, 0.11722425371408463],'ee_ori': [0.02249183878302574, 0.9232774972915649, -0.3816472887992859, -0.03739221394062042], 
#'fingertip_pos': [0.6397152476478642, 0.00610617024250492, 0.005252275150210414], 'fingertip_ori': [0.006470368725654669, 0.9990472770064632, 0.0007268829289016066, -0.043153155200712945]}