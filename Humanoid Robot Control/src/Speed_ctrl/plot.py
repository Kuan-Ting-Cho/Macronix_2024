import matplotlib.pyplot as plt
import numpy as np
import csv
import pandas as pd
index={0:0,1:9,2:1,3:2,4:3,5:10    #11,12,13,14,15,16,21,22,23,24,25,26
      ,6:4,7:11,8:5,9:6,10:7,11:12}
# 將弧度換算角度進行控制運算
def rad2deg(radius):
    return radius/np.pi*180
joint_name = ['L_hip_yaw', 'L_hip_roll', 'L_hip_pitch', 'L_knee', 'L_ankle_pitch', 'L_ankle_roll', 'R_hip_yaw', 'R__hip_roll', 'R__hip_pitch', 'R__knee', 'R__ankle_pitch', 'R__ankle_roll']
bias=np.array([32,65,186.7,29.1,39,138.3,204.3,45.5,0,-1,0,1.5,0])# 11,13,14,15,21,23,24,25,1,12,16,22,26


r_motion = pd.read_csv("Speed_ctrl/return.csv",header=None,index_col=None)
d_motion  = pd.read_csv("Speed_ctrl/desired.csv",header=None,index_col=None)
r_motion = r_motion.iloc[:,1:].T.reset_index(drop=True).T
#print每顆馬達的角度圖
for i in range(0,12):
    plt.subplot(2, 6, i+1)
    d_motion[i+1].plot()
    r_motion[i].plot()
    plt.legend(['desired','real'])
    plt.xlabel('Timestep')
    plt.ylabel('Degree')
    plt.title(joint_name[i])

plt.show()
