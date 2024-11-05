from src.LIPMMotionGenerator import LIPM_motion_generator
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataprocess import Data_preprocess
import time
from Cho.lipm_5 import *
from Cho.lipm_7 import *
from Cho.lipm_6 import *
def deg2rad(angle):
    # global np.pi
    return angle*np.pi/180
def lipm_FSF(total_step,generate_step,desired_step,desired_one_step_time,init_l,init_r):
    balance_step=50
    if desired_one_step_time[0]==0.5:
        lipm_5(total_step,8,init_l,init_r)
    elif desired_one_step_time[0]==0.6:
        lipm_6(total_step,8,init_l,init_r)
    elif desired_one_step_time[0]==0.7:
        lipm_7(total_step,8,init_l,init_r)
    motion_F = pd.read_csv('Cho/src/F2.csv',header=None,index_col=None)
    # motion_F = Data_preprocess(motion_F,balance_step)

    if desired_one_step_time[1]==0.5:
        lipm_5(total_step,8,init_l,init_r)
    elif desired_one_step_time[1]==0.6:
        lipm_6(total_step,8,init_l,init_r)
    elif desired_one_step_time[1]==0.7:
        lipm_7(total_step,8,init_l,init_r)
    motion_S = pd.read_csv('Cho/src/F2.csv',header=None,index_col=None)
    # motion_S=Data_preprocess(motion_S,balance_step)
    # print("Hi")
    # print(len(motion_F),len(motion_S))
    s_step=int(desired_one_step_time[0]/0.01)
    f_step=int(desired_one_step_time[1]/0.01)
    ## motion_F
    motion_F_1 = motion_F.iloc[0:50+2*s_step,:]
    motion_F_2 = motion_F.iloc[50+2*s_step:50+4*s_step,:]
    motion_F_3 = motion_F.iloc[50+4*s_step:50+6*s_step,:]
    motion_F_4 = motion_F.iloc[50+6*s_step:,:]
    motion_F_2_concat1 = pd.DataFrame()
    for i in range(int((desired_step[0]-2)/2)):
        motion_F_2_concat1 = pd.concat([motion_F_2_concat1,motion_F_2],axis=0,ignore_index=True)

    ## motion_S
    motion_S_1 = motion_S.iloc[0:50+2*f_step,:]
    motion_S_2 = motion_S.iloc[50+2*f_step:50+4*f_step,:]
    motion_S_3 = motion_S.iloc[50+6*f_step:,:]
    motion_S_2_concat1 = pd.DataFrame()
    for i in range(int((desired_step[1])/2)):
        motion_S_2_concat1 = pd.concat([motion_S_2_concat1,motion_S_2],axis=0,ignore_index=True)
    
    motion_F_2_concat2 = pd.DataFrame()
    for i in range(int((desired_step[2]-2)/2)):
        motion_F_2_concat2 = pd.concat([motion_F_2_concat2,motion_F_3],axis=0,ignore_index=True)

    motion=pd.concat([motion_F_1,motion_F_2_concat1,motion_S_2_concat1,motion_F_2_concat2,motion_F_4],axis=0,ignore_index=True)

#dynamix delay
    dynamix=[2,6,8,12]
    for idx in dynamix:
        num = 7
        temp = [0]*num+list(motion.iloc[ :,idx])
        temp = temp[0:len(temp)-num]
        motion[idx] = pd.DataFrame(temp,columns=[idx])
        motion.iloc[len(motion)-100:len(motion),idx] = [0]*len(motion.iloc[len(motion)-100:len(motion),idx])

    
    motion.iloc[:,2]-=deg2rad(init_l[0])
    motion.iloc[:,3]-=deg2rad(init_l[1])
    motion.iloc[:,4]-=deg2rad(init_l[2])
    motion.iloc[:,5]-=deg2rad(init_l[3])
    motion.iloc[:,6]-=deg2rad(init_l[4])

    motion.iloc[:,8]-=deg2rad(init_r[0])
    motion.iloc[:,9]-=deg2rad(init_r[1])
    motion.iloc[:,10]-=deg2rad(init_r[2])
    motion.iloc[:,11]-=deg2rad(init_r[3])
    motion.iloc[:,12]-=deg2rad(init_r[4])

    pd.DataFrame(motion).to_csv('Cho/src/F2_filter.csv',header=None,index=False)