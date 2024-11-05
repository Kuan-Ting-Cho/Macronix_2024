import numpy as np
import pandas as pd
from scipy.stats import norm
pi = np.pi

# 機器人估重 19.74 Kg
def deg2rad(angle):
    global pi
    return angle*pi/180
# 將弧度換算角度進行控制運算
def rad2deg(radius):
    global pi
    return radius/pi*180
# 將Motion_data做線性插值，回傳插值動作
def Linear_interp(motion,index1,index2,num):
    motion1=motion.loc[index1].tolist()
    motion2=motion.loc[index2].tolist()
    motion_interp = np.linspace(motion1, motion2, num)
    return motion_interp
# 將Motion_data做線性插值，回傳加入插值動作的新Motion_data
def Data_preprocess(motion,footstep):
    # 左右腳對調
    motion_left=motion.iloc[:,6:]
    motion_right=motion.iloc[:,0:6]
    motion=pd.concat([motion_left,motion_right],axis=1,ignore_index=True)
    pd.DataFrame(motion).to_csv('Chaochi/src/F2_origin.csv',header=None,index=False)
    # 複製初始蹲姿(這邊給50 time steps)
    # df1 = pd.DataFrame(Linear_interp(motion,0,0,balance_step))
    # motion=pd.concat([df1,motion],axis=0,ignore_index=True)
    # motion = motion.drop(balance_step).reset_index(drop=True)

    #加入走完兩步平衡所需的Step(這邊給50 time steps)
    # df2 = pd.DataFrame(Linear_interp(motion,len(motion.iloc[:,0])-1,len(motion.iloc[:,0])-1,balance_step))
    # motion=pd.concat([motion,df2],axis=0,ignore_index=True)
    motion = np.array(motion)
    for i in range(1,len(motion)):
        if motion[i][3]<motion[0][3]:
            motion[i][3]=motion[0][3]
        if motion[i][9]<motion[0][9]:
            motion[i][9]=motion[0][9]
    
    for i in range(1,45):
        if motion[i][4]>motion[0][4]:
            motion[i][4]=motion[0][4]
    
    for i in range(82,140):
        if motion[i][10]>motion[139][10]:
            motion[i][10]=motion[139][10]
        if i>100 and i<135:
            motion[i][10]-=deg2rad(0.5)
        # if i>110 and i<135:
        #     motion[i][10]-=deg2rad(1)

    motion=np.transpose(motion)
    
    delay= 3 #以前是7

    for i in range(len(motion)):
        # if i == 4 or i == 10:
        #     idx=np.argmin(motion[i])
        #     motion[i][idx-num:idx]=np.array([motion[i][idx]]*num)
        if i == 2:
            num=19
            idx=np.argmax(motion[i])
            motion[i][idx+1:idx+num+1]=np.array([motion[i][idx]]*num)
        # if i == 8:
        #     num=10
        #     idx=np.argmin(motion[i])
        #     motion[i][idx-9:idx+num-9]=np.array([motion[i][idx]]*num)
        if i == 3:
            num=31
            move=21
            idx=np.argmax(motion[i])
            motion[i][idx-num+move:idx+move]=np.array([motion[i][idx]]*num)
        if  i == 9:
            num=15
            idx=np.argmax(motion[i])
            motion[i][idx-num:idx]=np.array([motion[i][idx]]*num)
    motion=np.transpose(motion)
    motion= motion[:int(len(motion)*footstep/2)]
    for i in range(30):
        motion= np.append(motion,np.array([motion[0]]),axis=0)
    motion=np.transpose(motion)
    for i in range(len(motion)):
        if i == 1 or i == 5 or i == 7 or i == 11:
            motion[i]=np.array([motion[i][0]]*delay+list(motion[i][:len(motion[i])-delay]))
    # motion[5]*=1.1
    # motion[11]*=1.1
    #         motion[i]*=-1
    # motion_left=motion[:6]
    # motion_right=motion[6:]
    # motion=np.append(motion_right,motion_left,axis=0)

    motion=np.transpose(motion)
    pd.DataFrame(motion).to_csv('Chaochi/src/F2_200.csv',header=None,index=False)
    return motion
