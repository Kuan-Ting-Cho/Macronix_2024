import math
import numpy as np
import csv
from scipy.spatial.transform import Rotation as Rt
import sys
sys.path.append('Chaochi')
from inverse_kinematics import *
from math import sin
import matplotlib.pyplot as plt
np.set_printoptions(precision=4)
import time

pi = math.pi

# robot完全站直pos的相對座標
right=np.array([-0.0115,-0.00725,-1.0307])
left=np.array([-0.0115,-0.00725,-1.0307])



def deg2rad(angle):
    global pi
    return angle*pi/180

def rad2deg(radius):
    global pi
    return radius/pi*180


# 將弧度換算角度進行控制運算
def PD_controller(kp, kv, qpos, qvel, controller):
    signal = []
    for i in range(len(controller)):
        signal.append(-kp*(qpos[i]-controller[i])-kv*qvel[i])
        if signal[-1]>35: signal[-1]=35
        elif signal[-1]<-35: signal[-1]=-35
    return signal

# unit: mm
def zmp_pos(sensor):
    dx = 0.2
    dy = 0.12

    Rh_tua = (sensor[0]+sensor[2])/(sensor[1]+sensor[3]+1e-10)
    Rv_tua = (sensor[0]+sensor[1])/(sensor[2]+sensor[3]+1e-10)
    Ry = dy*Rh_tua/(1+Rh_tua)-dy/2
    Rx = dx*Rv_tua/(1+Rv_tua)-dx/2
    Lh_tua=(sensor[4]+sensor[6])/(sensor[5]+sensor[7]+1e-10)
    Lv_tua= (sensor[4]+sensor[5])/(sensor[6]+sensor[7]+1e-10)
    Ly = dy*Lh_tua/(1+Lh_tua)-dy/2
    Lx = dx*Lv_tua/(1+Lv_tua)-dx/2
    return np.array([Lx,Ly,Rx,Ry])*1000

def ZMP_controller(state,kp,kd):
    # return (kp*(state[-1]/1000)+kd*(state[-1]-state[-2])/1000)
    return (kp*(state[-1])+kd*(state[-1]-state[-2]))

def get_zmp_weight():
    file = open('Chaochi/zmp_best.csv')
    reader = csv.reader(file)
    weight = list(reader)[0]
    weight = [float(i) for i in weight]
    file.close()
    return weight

def get_PD_wieght():
    file = open('weight/PD.csv')
    reader = csv.reader(file)
    weight = list(reader)[0]
    weight = [float(i) for i in weight]
    weight[-1]=weight[-1]/10000
    file.close()
    return weight

def quat2euler(q):
    w,x,y,z=q
    r=Rt.from_quat([x,y,z,w])
    R_m=r.as_matrix()
    r=list(r.as_euler("xyz"))
    return r,R_m

def euler2quat(roll, pitch, yaw):
    e=Rt.from_euler('xyz',[roll, pitch, yaw])
    x,y,z,w=list(e.as_quat())
    return np.array([w,x,y,z])

def Rot_matrix(euler):
    e=Rt.from_euler('xyz',euler)
    R_m=e.as_dcm()
    return R_m

def q_weight(queue):
    return queue[-4]*0.1+queue[-3]*0.2+queue[-2]*0.3+queue[-1]*0.4

def imu_q_update(imu_queue,imu,angle,standar):
    for i,q in enumerate(imu_queue):
        q.append(imu[i]-standar[i])
        q.pop(0)
        angle[i].append(q_weight(q))
        angle[i].pop(0)
    return imu_queue,angle

def zmp_q_update(zmp_queue,zmp,record):
    for i,q in enumerate(zmp_queue):
        q.append(zmp[i])
        q.pop(0)
        record[i].append(q_weight(q))
        # record[i].pop(0)
    return zmp_queue,record

#調initial pos
def bal_pos(desired,R=[[1, 0, 0],[0, 1, 0],[0, 0, 1]],roll=0):
    adjust=np.array([0,0,0.105*sin(roll)]) # 板子平衡roll方向補償
    desired[1:7]=InvK_CHI(left+np.array([-0.025,0,0.054])+adjust,R) # 腳往後2.5cm,往上5.4cm
    desired[7:]=InvK_CHI(right+np.array([-0.025,0,0.054])-adjust,R)
    
    # 開機微調
    desired[3]-=2.5
    desired[4]-=2
    desired[5]-=1 #3
    # desired[8]+=1
    desired[9]-=0
    desired[10]-=10
    desired[11]+=-3 #3.5
    return desired

step = 0
acc=[0,0]
acti=[0,0]
times=[0,0]
last=[0,0]
def balance_algo(rot,pos,threshold=0.3,weight=[0.018,0.3,0.018,0.3]):
    global step
    global acc
    global acti
    global times
    global last
    rot=np.array(rot)
    rot[2]=0 # 不考慮yaw方向

    # roll方向傾角pd補償
    if abs(rot[0])<threshold:
        last[0]=rot[0]
        rot[0]=0
        acc[0]=0
    else:
        acc[0]=rot[0]
        if acc[0]>0:
            times[0]=acc[0]+0.65
            acti[0] += (weight[0]*times[0]+(rot[0]-last[0])*weight[1])
            acc[0] -= (weight[0]*times[0]+(rot[0]-last[0])*weight[1])
        elif acc[0]<0:
            times[0]=-acc[0]+0.65
            acti[0] -= (weight[0]*times[0]+(rot[0]-last[0])*weight[1])
            acc[0] += (weight[0]*times[0]+(rot[0]-last[0])*weight[1])
        last[0]=rot[0]

    # pitch方向傾角pd補償
    if abs(rot[1])<threshold:
        last[1]=rot[1]
        rot[1]=0
        acc[1]=0
    else:
        acc[1]=rot[1]
        if acc[1]>0:
            times[1]=acc[1]+0.2
            acti[1] += (weight[2]*times[1]+(rot[1]-last[1])*weight[3])
            acc[1] -= (weight[2]*times[1]+(rot[1]-last[1])*weight[3])
        elif acc[1]<0:
            times[1]=-acc[1]+0.2
            acti[1] -= (weight[2]*times[1]+(rot[1]-last[1])*weight[3])
            acc[1] += (weight[2]*times[1]+(rot[1]-last[1])*weight[3])
        last[1]=rot[1]
    rot[0]=acti[0]
    rot[1]=acti[1]
    rot=deg2rad(rot)
    R=Rot_matrix(rot)
    return rot,R

def loaded_para(base_rot,start,last_rot,bias):
    if abs(base_rot) > abs(last_rot):
        start = 5
        last_rot = base_rot
    if abs(base_rot) < deg2rad(0.05):
        pass
    elif start == 50:
        bias+=last_rot/2
        start = 1
        last_rot = base_rot
    else:
        start+=1
    return start,last_rot,bias

r_count=0
p_count=0
rflag=0
pflag=0
stop=0
# 寫死 static impact testing
def zmp_forcecontroll(RP):
    global r_count,p_count,rflag,pflag,pi,stop
    if RP == "r":
        if r_count<200:
            r_count+=1
        else:
            r_count=0
            rflag=0
        # print(2*sin(pi/200*r_count))
        return 2*sin(pi/200*r_count)
    if RP == "p" and not stop:
        if p_count>100 and p_count<200:
            p_count+=1
        if p_count<200:
            p_count+=5
        else:
            p_count=0
            pflag=0
            stop = 1
        return 15*sin(pi/100*p_count)
    

crtl_index = [6,7, 8, 9, 12, 13, 14, 17, 18, 19, 22, 23, 24]
record = [[0.0]*4,[0.0]*4,[0.0]*4,[0.0]*4] # for zmp 
standar = [-1000,-1000,0]
qlen=4
imu_queue = [[0.0]*qlen,[0.0]*qlen,[0.0]*qlen]
angle_q=[[0.0]*10,[0.0]*10,[0.0]*10]
zmp_queue=[[0]*qlen,[0]*qlen,[0]*qlen,[0]*qlen] # Lx,Ly,Rx,Ry
zmp_w=get_zmp_weight()
thres_load=250  # loaded balance
loaded=1        # loaded balance
last_rot=0.0    # loaded balance
bias = 0.0      # loaded balance
hip_adj = 0.0   # loaded balance
knee_adj=0.0    # loaded balance
step = 0

def balance_angle(desired,imu_data,pos,l_fsr,r_fsr,iszmp=False,isload=False):
    global step
    step+=1
    global record
    global qlen
    global imu_queue
    global zmp_queue
    global zmp_w
    global hip_adj
    global knee_adj
    global thres_load
    global loaded
    global last_rot
    global bias
    global standar
    global angle_q
    global rflag
    global pflag , stop

    # 修正IMU初始值
    if standar[0]==-1000 and standar[1]==-1000:
        standar[0]=imu_data[0]   
        standar[1]=imu_data[1]+1 # 向前傾1度

    # print(imu_data)
    # imu濾波
    imu_queue,angle_q=imu_q_update(imu_queue,imu_data,angle_q,standar)
    print(angle_q[0][-1],angle_q[1][-1],angle_q[2][-1])

    # 負責static impact testing
    # 走路時iszmp=1
    if iszmp:
        desired = bal_pos(desired)
        zmp=zmp_pos([r_fsr[3],r_fsr[1],r_fsr[2],r_fsr[0],l_fsr[3],l_fsr[1],l_fsr[2],l_fsr[0]]) # ZMP數值
        zmp_queue,record=zmp_q_update(zmp_queue,zmp,record) # ZMP濾波

        # 寫死static impact testing
        # if abs(angle_q[0][-1])>2:
        #     rflag=1
        # if abs(angle_q[1][-1])>2 and not stop:
        #     pflag=1
        # if rflag:
        #     desired[6]  += zmp_forcecontroll("r")
        #     desired[12] +=zmp_forcecontroll("r")
        # if pflag:
        #     desired[5]  += zmp_forcecontroll("p")
        #     desired[11] +=zmp_forcecontroll("p")

    # 靜態負重
    elif isload:
        rot,R=balance_algo([angle_q[0][-1],angle_q[1][-1],angle_q[2][-1]],pos,weight=[0.01,0.15,0.009,0.15]) # 馬達位置
        desired = bal_pos(desired,R,rot[0])
        # if abs(angle_q[1][-1])>2 and not stop:
        #     pflag=1
        # if pflag:
        #     desired[5]  += zmp_forcecontroll("p")
        #     desired[11] +=zmp_forcecontroll("p")
    else:
        # 板子上平衡
        rot,R=balance_algo([angle_q[0][-1],angle_q[1][-1],angle_q[2][-1]],pos) # 馬達位置
        desired = bal_pos(desired,R,rot[0])
 
    return desired

    