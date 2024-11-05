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
def Data_preprocess(motion,balance_step):
    # 左右腳對調
    motion_left=motion.iloc[:,6:]
    motion_right=motion.iloc[:,0:6]
    motion=pd.concat([motion_left,motion_right],axis=1,ignore_index=True)
    # 加一全為零的新欄(for trunk)
    df = pd.DataFrame([deg2rad(0)] * len(motion.iloc[:, 0]))
    motion = pd.concat([df, motion], axis=1, ignore_index=True)
    # 複製初始蹲姿(這邊給50 time steps)
    df1 = pd.DataFrame(Linear_interp(motion,0,0,balance_step))
    motion=pd.concat([df1,motion],axis=0,ignore_index=True)
    motion = motion.drop(balance_step).reset_index(drop=True)

    #加入走完兩步平衡所需的Step(這邊給50 time steps)
    df2 = pd.DataFrame(Linear_interp(motion,len(motion.iloc[:,0])-1,len(motion.iloc[:,0])-1,balance_step*2))
    motion=pd.concat([motion,df2],axis=0,ignore_index=True)

    # pd.DataFrame(motion).to_csv('localcom/Cho/F2.csv',header=None,index=False)
    return motion
