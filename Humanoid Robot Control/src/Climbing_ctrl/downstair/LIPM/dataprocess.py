import transforms3d as tfs
import numpy as np
import math
import pandas as pd
from scipy.stats import norm
pi = math.pi
# 機器人估重 19.74 Kg
def deg2rad(angle):
    global pi
    return angle*pi/180
# 將弧度換算角度進行控制運算
def rad2deg(radius):
    global pi
    return radius/pi*180

def limit(sig,threshold):
    if np.isnan(sig):
        sig=0
    elif sig > threshold:
        sig = threshold
    elif sig < -threshold:
        sig = -threshold
    return sig
def PID_control(kp, kv, ki, qpos, qvel, controller,acc_err,rise):
    signal = []
    for i in range(len(controller)):
        acc_err[i] = acc_err[i] + qpos[i]-controller[i]
        if i == 6 and rise == 2:
            sig=limit(-kp[1]*(qpos[i]-controller[i])-ki[1]*acc_err[i]-kv[1]*qvel[i],35)
            signal.append(sig)
        elif i == 12 and rise == 1:
            sig=limit(-kp[1]*(qpos[i]-controller[i])-ki[1]*acc_err[i]-kv[1]*qvel[i],35)
            signal.append(sig)
        else:
            sig=limit(-kp[0]*(qpos[i]-controller[i])-ki[0]*acc_err[i]-kv[0]*qvel[i],35)
            signal.append(sig)

    return signal,acc_err

def PD_control(kp, kv, qpos, qvel, controller):
    signal = []
    for i in range(len(controller)):
        signal.append(-kp*(qpos[i]-controller[i])-kv*qvel[i])
    return signal
    
# 將Motion_data做線性插值，回傳插值動作
def Linear_interp(motion,index1,index2,num):
    motion1=motion.loc[index1].tolist()
    motion2=motion.loc[index2].tolist()
    motion_interp = np.linspace(motion1, motion2, num)

    return motion_interp

# 將Motion_data做線性插值，回傳加入插值動作的新Motion_data
def Data_preprocess(motion,balance_step):
    # 左右腳對調
    motion_left=motion.iloc[:,6:]
    motion_right=motion.iloc[:,0:6]
    motion=pd.concat([motion_left,motion_right],axis=1,ignore_index=True)
    # 加一全為零的新欄(for trunk)
    df = pd.DataFrame([deg2rad(0)]*len(motion.iloc[:, 0]))
    motion_ori=pd.concat([df,motion],axis=1,ignore_index=True)
    # 加一全為零的新列(站直)
    df1 = pd.DataFrame([0.0]*len(motion_ori.loc[0])).T
    motion=pd.concat([df1,motion_ori],axis=0,ignore_index=True)
    # 在站直與初始蹲姿間做插值
    df2 = pd.DataFrame(Linear_interp(motion,0,1,balance_step))
    # df3 = pd.DataFrame(Linear_interp(motion,1,1,1000)) #test
    # motion=pd.concat([df2,df3],axis=0,ignore_index=True) #test
    motion=pd.concat([df2,motion],axis=0,ignore_index=True)
    motion = motion.drop(balance_step).reset_index(drop=True)
    #加入走完兩步平衡所需的Step(這邊給150 time steps)
    df3 = pd.DataFrame(Linear_interp(motion,len(motion.iloc[:,0])-1,len(motion.iloc[:,0])-1,1000))
    motion=pd.concat([motion,df3],axis=0,ignore_index=True)
    # #回到初始蹲姿(這邊給50 time steps)
    # df4 = pd.DataFrame(Linear_interp(motion,len(motion.iloc[:,0])-1,balance_step,balance_step))
    # motion=pd.concat([motion,df4],axis=0,ignore_index=True)
    return motion,motion_ori
# 尋找關節step突跳點
def find_inflection_points_and_max_value(data,balance_step):
    # 查找转折点
    inflection_points = []
    print(len(data))
    for i in range(1, len(data) - 1):
        if i>30:
            if (data[i] < data[i - 1] and data[i] < data[i + 1]) or (data[i] > data[i - 1] and data[i] > data[i + 1]):
                inflection_points.append(i)
            elif data[i] == data[i - 1] and (data[i] > data[i + 1] or data[i] < data[i + 1]):
                inflection_points.append(i)
            elif (data[i] > data[i - 1] or data[i] < data[i - 1]) and data[i] == data[i + 1]:
                inflection_points.append(i)
    center_value = data[inflection_points[1]]-data[inflection_points[0]]
    inflection_points=[i+balance_step for i in inflection_points] # balance_step=站直與初始蹲姿間插值的step
    return inflection_points, center_value

def generate_normal_distribution(motion,balance_step,extend_step): 
    point,center_value=find_inflection_points_and_max_value(motion[balance_step:].tolist(),balance_step)
    print(point,center_value)
    step=point[2]-point[0] #step=突波的step
    # 生成正態分布數據
    mean=0
    std_dev=1
    x = np.linspace(mean - 3 * std_dev, mean + 3 * std_dev, step+extend_step)
    pdf = norm.pdf(x, loc=mean, scale=std_dev)
    # 根據中間值縮放數據
    scaled_pdf=pdf*center_value / max(pdf)
    # 將轉折點頭尾擴大
    point[0]=point[0]-int(extend_step/2)
    point[2]=point[2]+int(extend_step/2)
    return point,scaled_pdf

def Sole_Status(sensordata):
    parallel=[]
    for idx in range(0,len(sensordata),2):
        if sensordata[idx]!=0 or sensordata[idx+1]!=0:
           parallel.append(1)
        else: 
           parallel.append(0)
    return parallel
