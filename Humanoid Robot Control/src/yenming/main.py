import sys
import os
sys.path.append(os.path.abspath('/home/airobots/node/src'))
from robot.scripts.cmd import *
from test import *
import threading
from lipm import *
from function import *
import numpy as np
import subprocess
import math
import csv

status=np.ones((16,4),dtype='float32')
pos=[0.0]*13
l_fsr=[0.0]*4
r_fsr=[0.0]*4
IMU_data=[0,0,0]
step =0
pi = math.pi

def deg2rad(angle):
    global pi
    return angle*pi/180

def rad2deg(radius):
    global pi
    return radius/pi*180

def get_status():  
    global status,pos,l_fsr,r_fsr,IMU_data
    while True:
        status,pos,_=update_status()
        IMU_data=status[-3][:3]
        l_fsr=status[-2]
        r_fsr=status[-1]
        
def main():
    # send command using send_cmd, input:desired 
    global pos,IMU_data,l_fsr,r_fsr,step
    desired=[0.0]*13
    walk=1
    isflat=0 #是否為平地
    isbalance=1
    isterrain= 0 #是否承繼前個中止狀態
    count = 1    #5-向右轉
    count2 = 1
    chgini = 0

    
    aaa = [[0.0,0.0,0.0,0.0],   [-1.2,0.0,-1.2,0.0],  [-0.0,0.0,-1.1,-0.0], [-0.9,0.5,-0.0,-0.0],  [1.5,0.5,0.0,0.0], [0.5,0.0,1.8,1.0]]  #旺宏
    bbb = [[0.0,0.0,0.0,0.0],  [-2.7,-0.7,-2.7,0.0],[-4.8,-0.9,-4.8,0.0], [-3.2,0.0,-3.2,0.0], [-1.5,0.0,-1.5,0.0],[-4.8,0.5,-4.8,0.0],[-1,0.0,-1,0.0]]  #上坡
    ccc = [[0.0,0.0,0.0,0.0],  [0.0,-5.0,0.0,0.0],  [0.5,-5.0,0.0,0.0], [1.2,-5.0,0.0,0.0],[0.0,0.0,0.0,0.0]]  #單腳roll
    ddd = [[0.0,0.0,0.0,0.0],  [0.7,0.3,-0.0,0.5],[3.5,0.5,3.5,0.5-0.2],[3.5,0.5,1.5,0.0],[1.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    eee = [[0.0,0.0,0.0,0.0],  [0.7,0.3,-0.0,0.5],[3.5,0.5,3.5,0.5-0.2],[3.5,0.5,1.5,0.0],[1.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
    aa = [[-2.0,0.0,-2.0,0.0],[-3.9,-0.5,-3.9,0.0],[-5.1,-0.5,-5.1,0.0], [-5.9,-0.8,-5.9,0.0]]   #上坡
    bb = [[-0.0,-5.0,-0.0,0.0],[1.2,-5.0,-0.0,0.0], [1.5,-5.0,0.0,0.0], [1.0,-5.0,0.0,0.0]]  #單腳roll
    cc = [[-0.2,0.0,0.0,0.0],[0.7,0.3,0.0,0.9],[3.0,0.8-0.6,3.0,0.9-0.5],[2.5,0.5,1.5,0.0],[4.1,1.0,5.2,1.0],[3.5,1.0,2.5,0.0]]
    dd = [[-1.2,0.0,0.0,0.0],[2.0,0.0,2.4,0.0],[2.1,-0.4,3.2,0.0],[1.7,0.0,0.0,0.0]]
    finalterrain = bbb[count]
    if count != 0 and count!=1:
        isterrain=1

    #-----------------改初始動作--------------------------#
    if chgini:
        with open('yenming/src/finalterrain.csv') as csvFile : #開啟檔案
            csvReader = csv.reader(csvFile) #將檔案建立成Reader物件
            listReport = list(csvReader)
        firstterrain = listReport[0]
        for i in range(len(firstterrain)):
            firstterrain[i]=float(firstterrain[i])

        with open('yenming/src/F2_200.csv') as csvFile : #開啟檔案
            csvReader2 = csv.reader(csvFile) #將檔案建立成Reader物件
            listReport2 = list(csvReader2)
        finalposeterrain = listReport2[len(listReport2)-1]
        base = [float(0.0)]
        for i in range(len(finalposeterrain)):
            finalposeterrain[i]=float(finalposeterrain[i])
        finalposeterrain = np.append(base, finalposeterrain)
        finalposeterrain = np.array(finalposeterrain)
            
        LIPM_obj = Forward2()
        LIPM_obj.firstterrain = firstterrain  
        LIPM_obj.finalterrain  =   aa[count2]
        desired=rad2deg(finalposeterrain)
        for i in range(len(finalterrain)):
            isflat += finalterrain[i]
        LIPM_obj.output_motion(deg2rad(np.array(desired[1:])),2,isflat)
        motion = csv2cmd('yenming/src/F2_200.csv' )
        # time.sleep(10000000)
    #-----------------改初始動作--------------------------#

    # finalterrain = [-0.0,0.0,-0.0,-0.0] 
                    #(角度)左腳結束pitch、左腳結束roll、右腳結束pitch、右腳結束roll
                    #pitch:正：壓,  roll:正：左翻
    #-----------------讀前一個finalterrain--------------------------#
    with open('yenming/src/finalterrain.csv') as csvFile : #開啟檔案
        csvReader = csv.reader(csvFile) #將檔案建立成Reader物件
        listReport = list(csvReader)
    firstterrain = listReport[0]
    for i in range(len(firstterrain)):
        firstterrain[i]=float(firstterrain[i])
    #-----------------讀前一個finalterrain--------------------------#
    #-----------------讀前一個finalposeterrain--------------------------#
    with open('yenming/src/F2_200.csv') as csvFile : #開啟檔案
        csvReader2 = csv.reader(csvFile) #將檔案建立成Reader物件
        listReport2 = list(csvReader2)
    finalposeterrain = listReport2[len(listReport2)-1]
    base = [float(0.0)]
    for i in range(len(finalposeterrain)):
        finalposeterrain[i]=float(finalposeterrain[i])
    finalposeterrain = np.append(base, finalposeterrain)
    finalposeterrain = np.array(finalposeterrain)
    #-----------------讀前一個finalposeterrain--------------------------#
    if isbalance:
        while True:
            if isterrain==1:
                desired=rad2deg(finalposeterrain)
                send_cmd(desired)
            else:
                # 左腳：1234156
                # 右腳：789101112
                desired=rad2deg(np.array([0.0,0.0,-4.105519201891283e-05,-0.3701913986701699,0.687620062583475,-0.4134217727729931,4.105519201891283e-05,0.0,-4.105519201891283e-05,-0.3265581673703117,0.5479937224239286,-0.4483283578128798,4.105519201891283e-05]))
                desired[3]+=1-6 #+往後
                desired[4]+=0+1
                desired[5]+=-3#+壓
                desired[9]+=-1-6
                desired[10]+=6   #+彎曲
                desired[11]+=-1 +2#-抬       
                send_cmd(desired)
            # print(IMU_data)
            if step > 5:
                break
            step+=1
    time.sleep(1)
    footstep = 2
    if walk:
        LIPM_obj = Forward2()
        if isterrain==1:
            LIPM_obj.firstterrain = firstterrain
        else:
            LIPM_obj.firstterrain = [0,0,0,0]  
        LIPM_obj.finalterrain = finalterrain
        for i in range(len(finalterrain)):
            isflat += finalterrain[i]
        LIPM_obj.output_motion(deg2rad(np.array(desired[1:])),footstep,isflat)
        motion = csv2cmd('yenming/src/F2_200.csv' )
        print("desired=",desired)
        print("firstterrain=",LIPM_obj.firstterrain)
        print("finalterrain=",LIPM_obj.finalterrain)
        print("isflat=",isflat)
        for _ in range(1):
            step=0
            desired=[0.0]*13
            r_motion=[]
            d_motion=[]
            while step < len(motion):
                desired=Walk(step, desired, motion)
                r_motion.append(pos)
                d_motion.append(list(desired))
                # if step ==len(motion)-1:
                    # print("看清楚了:",desired)
                send_cmd(desired)
                    # print(step)
                if step==len(motion)-1:
                    print("plot")
                    r_motion = np.array(r_motion)
                    pd.DataFrame(r_motion).to_csv("yenming/return.csv",header=None,index=False)
                    pd.DataFrame(d_motion).to_csv("yenming/desired.csv",header=None,index=False)
                    msg1 = subprocess.Popen(['scp', 'yenming/return.csv', 'yenming/desired.csv', 'yenming/src/F2_origin.csv','airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/yenming'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                step+=1
            time.sleep(3)
    # while True:
    #     if step<300:
    #         desired=Walk(step, desired, motion)
    #         if reset:
    #             desired=[0.0]*13
    #             init_pos_l=[rad2deg(0.35)-1,rad2deg(0.7)+2.5,rad2deg(0.35)+3.5] #22,40,20
    #             init_pos_r=[rad2deg(0.35)+2,rad2deg(0.7)+1,rad2deg(0.35)+6.5] #22,42,21
    #         desired[3]-=init_pos_l[0]
    #         desired[4]+=init_pos_l[1]
    #         desired[5]-=init_pos_l[2]
    #         desired[9]-=init_pos_r[0]
    #         desired[10]+=init_pos_r[1]
    #         desired[11]-=init_pos_r[2]
    #         send_cmd(desired)
            
    #         r_motion.append(pos)
    #         d_motion.append(list(desired))
    #         # print(d_motion[-1])
    #         # print("\n")
    #     if step==299:
    #         print("plot")
    #         r_motion = np.array(r_motion)
    #         pd.DataFrame(r_motion).to_csv("yenming/return.csv",header=None,index=False)
    #         pd.DataFrame(d_motion).to_csv("yenming/desired.csv",header=None,index=False)
    #         msg1 = subprocess.Popen(['scp', 'yenming/return.csv', 'yenming/desired.csv', 'yenming/src/F2.csv','airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/cho'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    #         # msg2 = subprocess.Popen(['scp', 'Cho/desired.csv', 'airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Cho'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    #         # msg3 = subprocess.Popen(['scp', 'Cho/F2.csv', 'airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Cho'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    #         break
    #     step+=1

if __name__ == '__main__':
    activation()
    try:
        imu = threading.Thread(target = update_IMU)
        # fsr = threading.Thread(target = update_fsr)
        recv = threading.Thread(target = get_status)
        cmd = threading.Thread(target = main)
        imu.start()
        # fsr.start()
        recv.start()
        cmd.start()
        imu.join()
        # fsr.join()
        recv.join()
        cmd.join()
        
        print("end")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        
