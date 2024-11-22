#!/usr/bin/env python
#delay 6 steps
import rospy
from std_msgs.msg import String,Float32MultiArray,MultiArrayDimension
import threading
import numpy as np
import sys
sys.path.append('/home/airobots/Linkage_Robot_simulation/src')
from invk import *
import csv
import copy
import matplotlib.pyplot as plt
import pandas as pdccccccccccccccc
from Chaochi.balance import *
from Cho.function import *
from time import time as t

r_motor=[]
d_motor=[]
Step=0
index={11:0,13:1,14:2,15:3,21:4,23:5
      ,24:6,25:7,1:8,12:9,16:10,22:11,26:12}
bias=np.array([32,65,186.7,29.1,39,138.3,204.3,45.5,0,-1,0,1.5,0])# 11,13,14,15,21,23,24,25,1,12,16,22,26
limit=np.array([[51,-7],[140,55],[104.5,10],[76,-0.6],[80,22],[26,-66],[25,-70],[91,1.6]]) #要寫極限角判斷
imu_data=[0,0,0]
l_fsr=[0.0]*4
r_fsr=[0.0]*4
pos=[0.0]*13 # 對應desired
last_d=[0.0]*13

def csv2cmd(filename):
    file = open(filename)
    reader = csv.reader(file)
    data_list = list(reader)
    file.close()
    for i in range(len(data_list)):
        for j in range(len(data_list[i])):
            data_list[i][j] = float(data_list[i][j])
        data_list[i].insert(0,float(0)) #加入trunk角度
    return data_list

def dataprocess(filename):
    #讀資料
    initmotion = csv2cmd(filename)

    #左右腳馬達互換
    motionchg = copy.deepcopy(initmotion)
    for i in range(len(motionchg)):
        for j in range(1, len(motionchg[i])):
            if (j < 7):
                motionchg[i][j] = initmotion[i][j+6]
            else:
                motionchg[i][j] = initmotion[i][j-6]
            
            if (j == 2 ):
                motionchg[i][j] = motionchg[i][j]*1
            if (j == 6 ):
                motionchg[i][j] = motionchg[i][j]*1
            if (j == 8 ):
                motionchg[i][j] = motionchg[i][j]*1
            if (j ==12):
                motionchg[i][j] = motionchg[i][j]*1


    # 對動作檔進行插值(站立到準備動作)
    motion = []
    interpolation = 100
    for i in range(len(motionchg)):
        motioncol = [[0] * len(motionchg[i]) for _ in range(interpolation)]
        if i == 0:
            for j in range(len(motionchg[i])):
                temp1 = np.linspace(start = 0, stop = motionchg[i][j], num = interpolation)
                for k in range(interpolation):
                    motioncol[k][j] = temp1[k]
            for p in range(interpolation):
                motion.append(motioncol[p])
            for _ in range(50):     #準備姿勢前置多久
                motion.append(motioncol[interpolation-1])

        motion.append(motionchg[i])

    for i in range(len(motionchg)):
        motioncol = [[0] * len(motionchg[i]) for _ in range(interpolation)]
        if i == len(motionchg)-1:
            for j in range(len(motionchg[i])):
                temp1 = np.linspace(start = motionchg[i][j], stop = 0, num = interpolation)
                for k in range(interpolation):
                    motioncol[k][j] = temp1[k]
            for p in range(interpolation):
                motion.append(motioncol[p])

    return motion

def Squat_down(step,desired):
    right=np.array([-0.0115,-0.00725,-1.0307])
    left=np.array([-0.0115,-0.00725,-1.0307])
    if step > 100:
        time=step-100
        desired[1:7]=InvK(left+com_sin_trace(time,[0,0,0.05],500))
        desired[7:]=InvK(right+com_sin_trace(time,[0,0,0.05],500))
    return desired

def Swing(step,desired):
    right=np.array([-0.0115,-0.00725,-1.0307])
    left=np.array([-0.0115,-0.00725,-1.0307])
    if step > 200:
        time=step-200
        desired[1:7]=InvK(left+com_sin_trace(time,[0,0.1,0],500))
        desired[7:]=InvK(right+com_sin_trace(time,[0,0.1,0],500))
    return desired


def get_cmd(motor,desired):
    reset = 0
    global Step, pos, r_motor,d_motor
    init_pos_r=[0,0,0] #20 40
    init_pos_l=[0,0,0] #20 40
    if reset:
        desired=[0.0]*13
        init_pos_l=[22,40,20] #20 40
        init_pos_r=[22,42,21] #20 40 
    motor[index[11]][:3]=np.array([11,4,bias[0]],dtype=np.float64)
    motor[index[13]][:3]=np.array([13,4,-desired[9]+bias[1]+init_pos_r[0]],dtype=np.float64)
    motor[index[14]][:3]=np.array([14,4,desired[10]+bias[2]+init_pos_r[1]],dtype=np.float64)
    motor[index[15]][:3]=np.array([15,4,-desired[11]+bias[3]+init_pos_r[2]],dtype=np.float64)
    motor[index[21]][:3]=np.array([21,4,bias[4]],dtype=np.float64)
    motor[index[23]][:3]=np.array([23,4,desired[3]+bias[5]-init_pos_l[0]],dtype=np.float64)
    motor[index[24]][:3]=np.array([24,4,-desired[4]+bias[6]-init_pos_l[1]],dtype=np.float64)
    motor[index[25]][:3]=np.array([25,4,desired[5]+bias[7]-init_pos_l[2]],dtype=np.float64)
    motor[index[1]][:2]=np.array([1,desired[0]+bias[8]],dtype=np.float64)
    motor[index[12]][:2]=np.array([12,desired[8]+bias[9]],dtype=np.float64)
    motor[index[16]][:2]=np.array([16,desired[12]+bias[10]],dtype=np.float64)
    motor[index[22]][:2]=np.array([22,desired[2]+bias[11]],dtype=np.float64)
    motor[index[26]][:2]=np.array([26,desired[6]+bias[12]],dtype=np.float64)
    
    
    # for i in range(8):
    #     if limit[i][0]<motor[i][2]:
    #         motor[i][2]=limit[i][0]
    #     elif limit[i][1]>motor[i][2]:
    #         motor[i][2]=limit[i][1]
    
    if Step<300:
            r_motor.append(pos)
    if Step==299:
        print("plot")
        r_motor = np.array(r_motor)
        pd.DataFrame(r_motor).to_csv("localcom/Cho/return.csv",header=None,index=False)
    Step+=1
    
    return motor

def command():
    global imu_data
    global pos
    # MyActuator mode
    # 'shut_down': 0,  # torque off
    # 'stop': 1,   # still torque on
    # 'torque': 2,
    # 'speed': 3,
    # 'position': 4,
    # 'pos_increment': 5,
    # 'angle': 6,
    # 'status': 7,
    # 'reset': 8, # switch between motion and normal mode
    # 'torque_off': 9,
    # 'torque_on': 10
    # 'motion_cmd': 11
    pub = rospy.Publisher('command', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(50) # Hz
    step=0
    desired=np.array([0.0]*13)
    # motion = dataprocess('src/F2_train3.csv')  # YenMing Chen
    motion = csv2cmd('localcom/Cho/F2.csv' )  # Mr. Draw

    while not rospy.is_shutdown():
        # add control code & data proccess & algorithm
        # cmd [id,mode,target value] MyActuator
        # if mode == 11 cmd [id,mode,pos, vel, t_ff, kp=10.012, kd=1]
        # cmd [id,target value] dynamixal
        motor=-np.ones((13,7)) #[11,13,14,15,21,23,24,25,1,12,16,22,26]
        # desired=Squat_down(step,desired)
        desired=Walk(step, desired, motion)
        # desired=balance_angle(desired,imu_data,pos,l_fsr,r_fsr,0)
        
        motor = get_cmd(motor,desired)
        cmd = Float32MultiArray(None,list(motor.reshape(13*7)))
        
        pub.publish(cmd)
        rate.sleep()
        step+=1

def recieve():
    rate = rospy.Rate(50)
    rospy.Subscriber('sensor', Float32MultiArray, callback)
    while not rospy.is_shutdown():
        rate.sleep()

def callback(data):
    global imu_data
    global pos
    global l_fsr
    global r_fsr
    # recieve sensor data & motor status
    status=np.array(data.data).reshape(16,4)
    imu_data = status[-3][:3]
    l_fsr = status[-2]
    r_fsr = status[-1]
    pos = [status[index[1]][1]-bias[index[1]],status[index[21]][1]-bias[index[21]],status[index[22]][1]-bias[index[22]],
           status[index[23]][1]-bias[index[23]],-(status[index[24]][1]-bias[index[24]]),status[index[25]][1]-bias[index[25]],
           status[index[26]][1]-bias[index[26]],status[index[11]][1]-bias[index[11]],status[index[12]][1]-bias[index[12]],
           -(status[index[13]][1]-bias[index[13]]),status[index[14]][1]-bias[index[14]],-(status[index[15]][1]-bias[index[15]]),
           status[index[16]][1]-bias[index[16]]]
    # print(pos)
    # print(r_fsr)
    # print("Motor status")
    # print("ID: {}, Current: {} A, Speed: {} dps, Degree: {},Torque: {}".format(status[0],status[1],status[2],status[3],status[4],status[5]))
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

if __name__ == '__main__':
    try:
        rospy.init_node('commander', anonymous=True)
        sub = threading.Thread(target = recieve)
        pub = threading.Thread(target = command)
        sub.start()
        pub.start()
        sub.join()
        pub.join()
    except rospy.ROSInterruptException:
        pass
