import pandas as pd
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

pi = math.pi
d_motor=[]
delay_time=0

def deg2rad(angle):
    global pi
    return angle * pi / 180


def rad2deg(radius):
    global pi
    return radius / pi * 180

def quart2rpy(quart):
    w, x, y, z = quart
    try:
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return roll, pitch, yaw
    
    except:
        print("Q2R fail")
        return 0, 0, 0

def rpy2quart(r, p, y):
    rot = R.from_euler('xyz', [r,p,y], degrees=True)
    rot_quat = rot.as_quat()

    return np.array([rot_quat[3], rot_quat[0], rot_quat[1], rot_quat[2]])

def motion2deg(motion):
    for i in range(len(motion)):
        motion[i]=rad2deg(motion[i])

    
    return motion

def add_bias(desired, init_bias_L, init_bias_R):
    desired[2]-=init_bias_L[0]
    desired[3]-=init_bias_L[1]
    desired[4]+=init_bias_L[2]
    desired[5]-=init_bias_L[3]
    desired[6]-=init_bias_L[4]

    desired[8]-=init_bias_R[0]
    desired[9]-=init_bias_R[1]
    desired[10]+=init_bias_R[2]
    desired[11]-=init_bias_R[3]
    desired[12]-=init_bias_R[4]

    return desired


def filt_for7(step, desired, initial_time):
    global delay_time
    for i in range(len(desired)):
        
        if (i==9 or i==10 or i==11) and step > 0+initial_time-delay_time and step <= 17+initial_time-delay_time:
            desired[9]=-21.27
            desired[10]=42.56
            desired[11]=-26.28

        elif i==9 and step>=18+initial_time and step<50+initial_time:
            desired[i]=-33.38

        elif i==10 and step>=18+initial_time and step<34+initial_time:
            desired[i]=57.4

        # elif i==10 and step>=43+initial_time and step<122+initial_time:
        #     desired[i]=35.3

        elif i==11 and step>=18+initial_time and step<27+initial_time:
            desired[i]=-32.62

        elif i==4 and step>=70+initial_time and step<90+initial_time:
            desired[i]=41.72

        elif i==4 and step>=90+initial_time and step<115+initial_time:
            desired[i]=59.95

        elif i==3 and step>=100+initial_time and step<127+initial_time:
            desired[i]=-21.45

        # # 最後結束動作處理
        # elif (i==4 or i==10) and step>=138+initial_time and step<238+initial_time:
        #     desired[10]+=10
        #     desired[4]+=10
        # elif (i==3 or i==9) and step>=138+initial_time and step<240+initial_time:
        #     desired[3]-=10
        #     desired[9]-=10


    return desired

def filt_for2(step, desired, initial_time):
    global delay_time
    for i in range(len(desired)):
        
        if (i==9 or i==10 or i==11) and step > 0+initial_time-delay_time and step <= 17+initial_time-delay_time:
            desired[9]=-21.27
            desired[10]=42.56
            desired[11]=-26.28

        elif i==9 and step>=18+initial_time and step<50+initial_time:
            desired[i]=-33.38

        elif i==10 and step>=18+initial_time and step<34+initial_time:
            desired[i]=57.4

        # elif i==10 and step>=43+initial_time and step<122+initial_time:
        #     desired[i]=35.3

        elif i==11 and step>=18+initial_time and step<27+initial_time:
            desired[i]=-32.62

        elif i==4 and step>=70+initial_time and step<90+initial_time:
            desired[i]=41.72

        elif i==4 and step>=90+initial_time and step<115+initial_time:
            desired[i]=59.95

        elif i==3 and step>=100+initial_time and step<127+initial_time:
            desired[i]=-21.45

        # # 最後結束動作處理
        # elif (i==4 or i==10) and step>=138+initial_time and step<238+initial_time:
        #     desired[10]+=10
        #     desired[4]+=10
        # elif (i==3 or i==9) and step>=138+initial_time and step<240+initial_time:
        #     desired[3]-=10
        #     desired[9]-=10
    return desired

def filt_for3(step, desired, initial_time):
    global delay_time
    for i in range(len(desired)):
        
        if (i==9 or i==10 or i==11) and step > 0+initial_time-delay_time and step <= 17+initial_time-delay_time:
            desired[9]=-21.27
            desired[10]=42.56
            desired[11]=-26.28

        elif i==9 and step>=18+initial_time and step<50+initial_time:
            desired[i]=-33.38

        elif i==10 and step>=18+initial_time and step<34+initial_time:
            desired[i]=57.4

        # elif i==10 and step>=43+initial_time and step<122+initial_time:
        #     desired[i]=35.3

        elif i==11 and step>=18+initial_time and step<27+initial_time:
            desired[i]=-32.62

        elif i==4 and step>=70+initial_time and step<90+initial_time:
            desired[i]=41.72

        elif i==4 and step>=90+initial_time and step<115+initial_time:
            desired[i]=59.95

        elif i==3 and step>=100+initial_time and step<127+initial_time:
            desired[i]=-21.45

        # # 最後結束動作處理
        # elif (i==4 or i==10) and step>=138+initial_time and step<238+initial_time:
        #     desired[10]+=10
        #     desired[4]+=10
        # elif (i==3 or i==9) and step>=138+initial_time and step<240+initial_time:
        #     desired[3]-=10
        #     desired[9]-=10
    return desired

def Walk(step, desired, motion, bias_l, bias_r, initial_stable_time, floor):
    i=step
    ctrl = motion.loc[i].tolist()
    desired = motion2deg(ctrl)
    if step >= 240 and step < 300:
        BL = list([bias_l[0], bias_l[1], bias_l[2], bias_l[3]+1, bias_l[4]])
        BR = list([bias_r[0], bias_r[1], bias_r[2], bias_r[3]+1, bias_r[4]])
        desired = add_bias(desired, BL, BR)
    else:
        desired = add_bias(desired, bias_l, bias_r)
    
    if floor==1:
        desired = filt_for7(step, desired, initial_stable_time)
    elif floor==2:
        desired = filt_for2(step, desired, initial_stable_time)
    # elif floor==3:
    #     desired = filt(step, desired, initial_stable_time)
    else:
        desired = filt_for3(step, desired, initial_stable_time)


    return desired

# 將Motion_data做線性插值，回傳插值動作
def Linear_interp(motion,index1,index2,num):
    motion1=motion.loc[index1].tolist()
    motion2=motion.loc[index2].tolist()
    motion_interp = np.linspace(motion1, motion2, num)

    return motion_interp

# 將Motion_data做線性插值，回傳加入插值動作的新Motion_data
def Data_preprocess(motion, initial_stable_time, floor):
    # Swap left and right leg cmd
    columns_titles = [6,7,8,9,10,11,0,1,2,3,4,5]
    motion = motion.reindex(columns=columns_titles)

    # 加一全為零的新欄(for trunk)
    df = pd.DataFrame([deg2rad(0)]*len(motion.iloc[:, 0]))
    motion=pd.concat([df,motion],axis=1,ignore_index=True)

    # 複製初始蹲姿(這邊給50 time steps)
    df1 = pd.DataFrame(Linear_interp(motion,0,0,initial_stable_time))
    motion=pd.concat([df1,motion],axis=0,ignore_index=True)

    df2 = pd.DataFrame(Linear_interp(motion,len(motion)-1,len(motion)-1,50))
    df2[2]=0
    df2[8]=0
    motion=pd.concat([motion[:189],df2],axis=0,ignore_index=True)
    # print(motion)

    # 在final與初始蹲姿間做插值
    df4 = pd.DataFrame(Linear_interp(motion,len(motion)-1,0,200))
    motion=pd.concat([motion, df4],axis=0,ignore_index=True)

    for i in range(len(motion)):
        if i == 3:
            # 6公分-9
            # 8公分-5
            # 10公分-7
            if floor == 1:
                num = 10
            elif floor == 2:
                num = 6
            elif floor == 3:
                num = 2
            idx=np.argmax(motion[i])
            motion[i][idx+1:idx+num+1]=np.array([motion[i][idx]]*num)


    return motion

def modify_motion(motion):
    global delay_time

    dynamix=[2,6,8,12]
    for idx in dynamix:
        num = 10
        if idx == 2:
            a = [motion.iloc[50+69,idx]]*num
            temp = list(motion.iloc[0:50+69,idx])+a+list(motion.iloc[50+74:len(motion)-num,idx])

        else:
            a = [motion.iloc[50+70,idx]]*num
            temp = list(motion.iloc[0:50+70,idx])+a+list(motion.iloc[50+70:len(motion)-num,idx])
        motion[idx] = pd.DataFrame(temp,columns=[idx])
        
    pd.DataFrame(motion).to_csv('./Xiang/downstair/motordata/F2_200.csv',header=None,index=False)
    
    myact=[9,10,11]

    for idx in myact:
        if idx==9:
            num=0
        elif idx==10:
            num=0
            delay_time=num

        temp = list(motion.iloc[ num:len(motion.iloc[:,idx])-1,idx])+[motion.iloc[len(motion.iloc[:,idx])-1,idx]]*num
        motion[idx] = pd.DataFrame(temp,columns=[idx])
    return motion