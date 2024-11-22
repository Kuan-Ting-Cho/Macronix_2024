import pandas as pd
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

pi = math.pi


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


# 將Motion_data做線性插值，回傳插值動作
def Linear_interp(motion, index1, index2, num):
    motion1 = motion.loc[index1].tolist()
    motion2 = motion.loc[index2].tolist()
    motion_interp = np.linspace(motion1, motion2, num)

    return motion_interp


def Data_preprocess(motion):
    # Swap left and right leg cmd
    columns_titles = [6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5]
    motion = motion.reindex(columns=columns_titles)

    # 加一全為零的新欄(for trunk)
    df = pd.DataFrame([deg2rad(0)] * len(motion.iloc[:, 0]))
    motion = pd.concat([df, motion], axis=1, ignore_index=True)
    # 加一全為零的新列(站直)
    df1 = pd.DataFrame([0.0] * len(motion.loc[0])).T
    motion = pd.concat([df1, motion], axis=0, ignore_index=True)

    # 在站直與初始蹲姿間做插值
    df2 = pd.DataFrame(Linear_interp(motion, 0, 1, 100))
    motion = pd.concat([df2, motion], axis=0, ignore_index=True)
    motion = motion.drop(100).reset_index(drop=True)

    # 加入站直動作平衡所需的Step(這邊給30 time steps)
    df3 = pd.DataFrame(0, index=range(30), columns=range(13))
    motion = pd.concat([df3, motion], axis=0, ignore_index=True)

    # 在final與初始蹲姿間做插值
    df4 = pd.DataFrame(Linear_interp(motion,329,130,150))
    motion=pd.concat([motion, df4],axis=0,ignore_index=True)
    motion = motion.drop(329).reset_index(drop=True)        
    motion = pd.concat([motion, motion.iloc[[-1]]], axis=0, ignore_index=True)    

    return motion