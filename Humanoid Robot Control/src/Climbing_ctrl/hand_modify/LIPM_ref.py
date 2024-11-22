import numpy as np
import math
import sys
import os
sys.path.append(os.path.abspath('/home/airobots/node/src'))
import pandas as pd
import threading
import matplotlib.pyplot as plt
from LIPM.F2 import Forward2
from robot.scripts.cmd import *
from Xiang.hand_modify.function import *
import subprocess


column = ['trunk', 'L_hip_yaw', 'L_hip_roll', 'L_hip_pitch', 'L_knee', 'L_ankle_pitch', 'L_ankle_roll', 'R_hip_yaw', 'R_hip_roll', 'R_hip_pitch', 'R_knee', 'R_ankle_pitch', 'R_ankle_roll']
pd.set_option('display.max_rows', 10000)
pi = math.pi



status=np.ones((16,4),dtype='float32')
pos=[0.0]*13
l_fsr=[0.0]*4
r_fsr=[0.0]*4
IMU_data=[0,0,0]
def get_status():
    global status,pos,l_fsr,r_fsr,IMU_data
    while True:
        status,pos,_=update_status()
        IMU_data=status[-3][:3]
        l_fsr=status[-2]
        r_fsr=status[-1]
        
def main():
    floor = 1
    back = 0

    while floor<5:
        if floor==1:
            LIPM_obj = Forward2(0.06)
        elif floor==2:
            LIPM_obj = Forward2(0.08)
        # elif floor==3:
        #     LIPM_obj = Forward2(0.0)
        elif floor==3:
            LIPM_obj = Forward2(0.085)

        LIPM_obj.output_motion()

        initial_stable_time = 50
        motion_file = './Xiang/hand_modify/motordata/F2_200.csv'
        motion = pd.read_csv(motion_file, header=None,index_col=None)
        motion=Data_preprocess(motion, initial_stable_time) # 50 stable + 200 + 50 stable + 100 inital

        motion=modify_motion(motion)


        global pos
        d_motion = []
        r_motion = []
        step =0
        desired = [0.0]*13


        reset = 0

        period = 70
        
        footstep = 2
        if footstep < 2:
            excution_step = initial_stable_time + footstep*period
        else:
            excution_step = initial_stable_time + 2*period + 50-1

        init_pos_r=[0,0,0,0,0] #20 40
        init_pos_l=[0,0,0,0,0] #20 40
        # bias_l=[0, -1,  2, 2+2.4+0.8, 0] # 腳尖變小下壓
        # bias_r=[0, -2,  0, 4+2.4+0.8, 0]

            

        if not back:
            bias_l=[0,    2,  -3,   2+2, 0]
            bias_r=[0, -1.5,  -9,  -0.5+2, 0]
        else:        
            bias_l=[0,    2,  -3,   2+3, 0]
            bias_r=[0, -1.5,  -9,  -0.5+3, 0]

        # step = initial_stable_time+period
        while True:
            if step < excution_step:
                if step == initial_stable_time+period or step == initial_stable_time+2*period+49 or step==1 or step==3:
                    if step==1:
                        back=0
                        bias_l=[0,    2,  -3,   2+2, 0]
                        bias_r=[0, -1.5,  -9,  -0.5+2, 0]
                    print("go on step: ", step)
                    a=input(" ")
                    time.sleep(1.5)

                desired=Walk(step, desired, motion, bias_l, bias_r, initial_stable_time, floor)
                if reset:
                    desired=[0.0]*13
                    init_pos_l=[0+bias_l[0],rad2deg(0.35)+bias_l[1],rad2deg(0.7)+bias_l[2],rad2deg(0.35)+bias_l[3],0+bias_l[4]] #22,40,20
                    init_pos_r=[0+bias_r[0],rad2deg(0.35)+bias_r[1],rad2deg(0.7)+bias_r[2],rad2deg(0.35)+bias_r[3],0+bias_r[4]] #22,42,21
                
                    desired[2]-=init_pos_l[0]
                    desired[3]-=init_pos_l[1]
                    desired[4]+=init_pos_l[2]
                    desired[5]-=init_pos_l[3]
                    desired[6]-=init_pos_l[4]

                    desired[8]-=init_pos_r[0]
                    desired[9]-=init_pos_r[1]
                    desired[10]+=init_pos_r[2]
                    desired[11]-=init_pos_r[3]
                    desired[12]-=init_pos_r[4]
                send_cmd(desired)
                
                r_motion.append(pos)
                d_motion.append(list(desired))

            if step == excution_step-2:
                r_motion = np.array(r_motion)
                pd.DataFrame(r_motion).to_csv("Xiang/return.csv",header=None,index=False)
                pd.DataFrame(d_motion).to_csv("Xiang/desired.csv",header=None,index=False)
                msg1 = subprocess.Popen(['scp', 'Xiang/return.csv', 'Xiang/desired.csv', 'Xiang/hand_modify/motordata/F2_200.csv','airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Xiang'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                print("plot")

                break
            step+=1

        print("Now on stair: ", floor)    

        floor+=1
        a=input("Next")
        back=1
        time.sleep(0.5)

if __name__ == '__main__':
    activation()
    print("Go !! Attention ฅ^•ﻌ•^ฅ\n-(///￣皿￣)☞ ─═≡☆゜★\n                      █▇▆▅▄▃▂＿　")
    try:
        imu = threading.Thread(target = update_IMU)
        # fsr = threading.Thread(target = update_fsr)
        recv = threading.Thread(target = get_status)
        cmd = threading.Thread(target = main)
        imu.start()
        # fsr.start()
        recv.start()
        cmd.start()
        imu.join()
        # fsr.join()
        recv.join()
        cmd.join()
        
        print("end")


    except KeyboardInterrupt:
        print("KeyboardInterrupt")
