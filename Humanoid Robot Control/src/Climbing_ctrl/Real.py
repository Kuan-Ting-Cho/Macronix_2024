import sys
import os
sys.path.append(os.path.abspath('/home/airobots/node/src'))
from robot.scripts.cmd import *
from test import *
import numpy as np
from Xiang.function import *



class Robot:
    def __init__(self, filename) -> None:
        self.filename = filename


    
    def set_motion(self):
        # 讀Motion檔
        motion_csv = pd.read_csv(
            self.filename, header=None, index_col=None
        )

        self.motion = Data_preprocess(motion_csv)

    def add_bias(self, desired):
        init_bias_L=[2, -2, -1] #20 40
        init_bias_R=[2, 0, +2.5] #20 40
        desired[3]+=init_bias_L[0]
        desired[4]+=init_bias_L[1]
        desired[5]+=init_bias_L[2]
        desired[9]+=init_bias_R[0]
        desired[10]+=init_bias_R[1]
        desired[11]+=init_bias_R[2]

        return desired

    def go_sim1(self):
        for i in range(130):
            ctrl = self.motion.loc[i].tolist()
            ctrl = motion2deg(ctrl)
            desired = self.add_bias(ctrl)
            # print(desired)
            send_cmd(desired)

    def go_sim2(self, i):        
        ctrl = self.motion.loc[i].tolist()
        ctrl = motion2deg(ctrl)
        desired = self.add_bias(ctrl)
        # print(desired)
        send_cmd(desired)

        return True
    
    def squat_down(self, inverse=False):
        init = pd.DataFrame([[0,0,0,-0.35,0.7,-0.35,0,0,0,-0.35,0.7,-0.35,0]])
        downstair = pd.DataFrame([[0,0,0,-0.55,1.1,-0.55,0,0,0,-0.55,1.1,-0.55,0]])
        motion = pd.concat([init, downstair],axis=0,ignore_index=True)
        if not inverse:
            motion = Linear_interp(motion,0,1,100)
        else:
            motion = Linear_interp(motion,1,0,100)

        motion = pd.DataFrame(motion)

        for i in range(len(motion)):
            ctrl = motion.loc[i].tolist()
            ctrl = motion2deg(ctrl)
            desired = self.add_bias(ctrl)
            send_cmd(desired)
        print("Action complete")


    
    def real_obs(self):        
        base_info = rpy2quart(self.IMU_data[0], self.IMU_data[1], self.IMU_data[2]-self.yaw_bias)
        base_acc = np.array([0]*3)
        motor_pos = np.array(self.pos) # 13
        motor_vel = np.array(self.vel)

        return base_info, base_acc, motor_pos, motor_vel


        
