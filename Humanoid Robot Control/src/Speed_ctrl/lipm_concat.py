from src.LIPMMotionGenerator import LIPM_motion_generator
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataprocess import Data_preprocess
import time
from Speed_ctrl.lipm_5 import *
from Speed_ctrl.lipm_6 import *
from Speed_ctrl.lipm_7 import *

def deg2rad(angle):
    return angle*np.pi/180
def lipm_change(total_step,generate_step,desired_step,desired_one_step_time,init_l,init_r,mode):
    balance_step=100
    if desired_one_step_time[0]==1:
        lipm_5(total_step,generate_step,8,init_l,init_r)
        lipm_5_r(total_step,generate_step,8,init_l,init_r)
    elif desired_one_step_time[0]==1.2:
        lipm_6(total_step,generate_step,8,init_l,init_r)
        lipm_6_r(total_step,generate_step,8,init_l,init_r)    
    elif desired_one_step_time[0]==1.4:
        lipm_7(total_step,generate_step,8,init_l,init_r)
        lipm_7_r(total_step,generate_step,8,init_l,init_r)
    motion_F = pd.read_csv('Speed_ctrl/src/F2.csv',header=None,index_col=None)
    motion_F_l = pd.read_csv('Speed_ctrl/src/F2_filter.csv',header=None,index_col=None)
    motion_F_r = pd.read_csv('Speed_ctrl/src/F2_filter_r.csv',header=None,index_col=None)
    if desired_one_step_time[1]==1:
        lipm_5(total_step,generate_step,8,init_l,init_r)
        lipm_5_r(total_step,generate_step,8,init_l,init_r)
    elif desired_one_step_time[1]==1.2:
        lipm_6(total_step,generate_step,8,init_l,init_r)
        lipm_6_r(total_step,generate_step,8,init_l,init_r) 
    elif desired_one_step_time[1]==1.4:
        lipm_7(total_step,generate_step,8,init_l,init_r)
        lipm_7_r(total_step,generate_step,8,init_l,init_r)
    motion_S = pd.read_csv('Speed_ctrl/src/F2.csv',header=None,index_col=None)
    motion_S_l = pd.read_csv('Speed_ctrl/src/F2_filter.csv',header=None,index_col=None)
    motion_S_r = pd.read_csv('Speed_ctrl/src/F2_filter_r.csv',header=None,index_col=None)
    s_step=int(desired_one_step_time[0]/0.02)
    f_step=int(desired_one_step_time[1]/0.02)
    ## motion_F
    motion_F1 = motion_F.iloc[0:balance_step+2*s_step,:]
    motion_F2 = motion_F.iloc[balance_step+2*s_step:balance_step+4*s_step,:]
    # motion_F3 = motion_F.iloc[balance_step+4*s_step:balance_step+6*s_step,:]
    motion_F4 = motion_F.iloc[balance_step+6*s_step:,:]
    motion_fl = motion_F_l.iloc[balance_step+7*s_step:,:]
    motion_fr = motion_F_r.iloc[balance_step+6*s_step:balance_step+7*s_step:]
    motion_F2_concat1 = pd.DataFrame()
    for i in range(int((desired_step[0]-2)/2)):
        motion_F2_concat1 = pd.concat([motion_F2_concat1,motion_F2],axis=0,ignore_index=True)
    ## motion_S
    motion_S1 = motion_S.iloc[0:balance_step+2*f_step,:]
    motion_S2 = motion_S.iloc[balance_step+2*f_step:balance_step+4*f_step,:]
    # motion_S3 = motion_F.iloc[balance_step+4*f_step:balance_step+6*f_step,:]
    motion_S4 = motion_S.iloc[balance_step+6*f_step:,:]
    motion_sl = motion_S_l.iloc[balance_step+7*s_step:,:]
    motion_sr = motion_S_r.iloc[balance_step+6*s_step:balance_step+7*s_step:]
    motion_S2_concat1 = pd.DataFrame()
    for i in range(int((desired_step[1]-2)/2)):
        motion_S2_concat1 = pd.concat([motion_S2_concat1,motion_S2],axis=0,ignore_index=True)
    
    # change concat mode
    if mode == "FS":
        motion=pd.concat([motion_F1,motion_F2_concat1,motion_S2_concat1,motion_S4],axis=0,ignore_index=True)
        Cmd=[[1,0]]*(int((desired_step[0]-2)/2)*f_step)
        Cmd=Cmd.append([[1,2]]*(2*s_step))
        Cmd=Cmd.append([[0,2]]*(int((desired_step[1]-2)/2)*s_step))
        Cmd=sum(Cmd, [])
    elif mode == "SF":
        motion=pd.concat([motion_S1,motion_S2_concat1,motion_F2_concat1,motion_F4],axis=0,ignore_index=True)
        Cmd=[[0,0]]*(int((desired_step[0]-2)/2)*s_step)
        Cmd=Cmd.append([[0,1]]*(2*f_step))
        Cmd=Cmd.append([[1,1]]*(int((desired_step[1]-2)/2)*f_step))
        Cmd=sum(Cmd, [])
    elif mode == "FSF":
        motion_S2_concat1 = pd.DataFrame()
        for i in range(int(desired_step[1]/2)):
            motion_S2_concat1 = pd.concat([motion_S2_concat1,motion_S2],axis=0,ignore_index=True)
        motion_F2_concat2 = pd.DataFrame()
        for i in range(int((desired_step[2]-2)/2)):
            motion_F2_concat2 = pd.concat([motion_F2_concat2,motion_F2],axis=0,ignore_index=True)   
        motion=pd.concat([motion_F1,motion_F2_concat1,motion_S2_concat1,motion_F2_concat2,motion_F4],axis=0,ignore_index=True)
        Cmd=[[1,0]]*(int((desired_step[0]-2)/2)*f_step)
        Cmd=Cmd.append([[1,2]]*(2*s_step))
        Cmd=Cmd.append([[0,2]]*(int((desired_step[1]-2)/2)*s_step))
        Cmd=Cmd.append([[0,1]]*(2*f_step))
        Cmd=Cmd.append([[1,1]]*(int((desired_step[2]-2)/2)*f_step))
        Cmd=sum(Cmd, [])
    elif mode == "SFS":
        motion_F2_concat1 = pd.DataFrame()
        for i in range(int(desired_step[1]/2)):
            motion_F2_concat1 = pd.concat([motion_F2_concat1,motion_F2],axis=0,ignore_index=True)
        motion_S2_concat2 = pd.DataFrame()
        for i in range(int((desired_step[2]-2)/2)):
            motion_S2_concat2 = pd.concat([motion_S2_concat2,motion_S2],axis=0,ignore_index=True)  
        motion=pd.concat([motion_S1,motion_S2_concat1,motion_F2_concat1,motion_S2_concat2,motion_S4],axis=0,ignore_index=True)
        Cmd=[[0,0]]*(int((desired_step[0]-2)/2)*s_step)
        Cmd=Cmd.append([[0,1]]*(2*f_step))
        Cmd=Cmd.append([[1,1]]*(int((desired_step[1]-2)/2)*f_step))
        Cmd=Cmd.append([[1,2]]*(2*s_step))
        Cmd=Cmd.append([[0,2]]*(int((desired_step[2]-2)/2)*s_step))
        Cmd=sum(Cmd, [])

    
    #dynamix delay
    dynamix=[2,6,8,12]
    for idx in dynamix:
        num = 7
        temp = [0]*num+list(motion.iloc[ :,idx])
        temp = temp[0:len(temp)-num]
        motion[idx] = pd.DataFrame(temp,columns=[idx])
        motion.iloc[len(motion)-balance_step:len(motion),idx] = [0]*len(motion.iloc[len(motion)-balance_step:len(motion),idx])

    
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
    
    pd.DataFrame(motion).to_csv('Speed_ctrl/src/F2_filter.csv',header=None,index=False)
    pd.DataFrame(motion_fr).to_csv('Speed_ctrl/src/F2_filter_fr.csv',header=None,index=False) #fast right foot stop
    pd.DataFrame(motion_fl).to_csv('Speed_ctrl/src/F2_filter_fl.csv',header=None,index=False)
    pd.DataFrame(motion_sr).to_csv('Speed_ctrl/src/F2_filter_sr.csv',header=None,index=False) #slow left foot stop
    pd.DataFrame(motion_sl).to_csv('Speed_ctrl/src/F2_filter_sl.csv',header=None,index=False)
    return Cmd