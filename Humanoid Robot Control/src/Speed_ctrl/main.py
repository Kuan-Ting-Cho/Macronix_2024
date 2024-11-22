import sys
import os
sys.path.append(os.path.abspath('/home/airobots/node/src'))
from robot.scripts.cmd import *
from test import *
import threading
import numpy as np
from Speed_ctrl.function import *
from Speed_ctrl.lipm_5 import *
from Speed_ctrl.lipm_7 import *
from Speed_ctrl.lipm_6 import *
from Speed_ctrl.lipm_concat import *
import subprocess

status=np.ones((16,4),dtype='float32')
pos=[0.0]*13
distance=0.0
l_fsr=[0.0]*4
r_fsr=[0.0]*4
fsr_pos = [ [0.0755, -0.225],  # LFL
            [0.0755, -0.0750],  # LFR
            [-0.0545, -0.225],  # LBL
            [-0.0545, -0.0750],  # LBR
            [0.0755, 0.0750],  # RFL
            [0.0755, 0.225],  # RFR
            [-0.0545, 0.0750],  # RBL
            [-0.0545, 0.225]]  # RBR
IMU_data=[0.0]*3
CoP_data=[0.0]*2
def get_status():
    global status,pos,l_fsr,r_fsr,IMU_data,distance
    while True:
        status,pos,_=update_status()
        distance=status[-4][:1]
        IMU_data=status[-3][:3]
        l_fsr=status[-2]
        r_fsr=status[-1]
        
def main():
    # send command using send_cmd, input:desired angle
    global pos,IMU_data,l_fsr,r_fsr,CoP_data,distance
    d_motion = []
    r_motion = []
    IMU = [[0.0,0.0,0.0]]
    CoP = [[0.0,0.0],[0.0,0.0]]
    step = 0

    Cmd="FS"
    total_step=12
    generate_step=8
    desired_one_step_time=[1,1.4] #fast / slow
    desired_step=[4,4,4]
    period = [50,70,50] #sampling time 0.02s 

    if Cmd != "N" and total_step!=desired_step[0]+desired_step[1]+desired_step[2]:
        print("step unequal!!!!!!  error!!!!!")
        time.sleep(balance_step)

    reset = 0
    init_pos_l=[0]*5 #20 40
    init_pos_r=[0]*5 #20 40 
    init_roll=0
    init_l=[init_roll, 0+1, 2.5+1, 3,-init_roll]  # hip+ 前踢/knee+ 伸直/ankle - 下壓
    init_r=[init_roll, -3, 10.5, 9-2,-init_roll]
    if reset:
        init_pos_l=[0+init_l[0],rad2deg(0.3)+init_l[1],rad2deg(-0.6)+init_l[2],rad2deg(0.3)+init_l[3],0+init_l[4]] #22,40,20
        init_pos_r=[0+init_r[0],rad2deg(0.3)+init_r[1],rad2deg(-0.6)+init_r[2],rad2deg(0.3)+init_r[3],0+init_r[4]] #22,42,21
    if Cmd!="N":
        Cmd = lipm_change(total_step,generate_step,desired_step,desired_one_step_time,init_l,init_r,Cmd)
    elif period[0]==50:
        lipm_5(total_step,generate_step,init_l,init_r)
        Cmd = [[0,0]]*((total_step-2)*period[0])
    elif period[0]==60:
        lipm_6(total_step,generate_step,init_l,init_r)
        Cmd = [[0,0]]*((total_step-2)*period[0])
    elif  period[0]==70:
        lipm_7(total_step,generate_step,init_l,init_r)
        Cmd = [[0,0]]*((total_step-2)*period[0])

    motion = csv2cmd('Speed_ctrl/src/F2_filter.csv' )
    motion_fr = csv2cmd('Speed_ctrl/src/F2_filter_fr.csv')
    motion_fl = csv2cmd('Speed_ctrl/src/F2_filter_fl.csv')
    motion_sr = csv2cmd('Speed_ctrl/src/F2_filter_sr.csv')
    motion_sl = csv2cmd('Speed_ctrl/src/F2_filter_sl.csv')
    excution_step = len(motion)
    balance_step = 100
    trans_point=[balance_step+period[0]*desired_step[0],balance_step+period[0]*desired_step[0]+period[1]*desired_step[1]]
    stop = False # emergency_stop
    count = 0
    while True:
        if distance<=0.3 and count<1:
            stop = True # emergency_stop
            count+=1
        if stop==True:
            if step<=trans_point[0]:
                temp = (step-balance_step)//period[0]+1
                foot = temp % 2
                excution_step=balance_step+temp*period[0]
                if period[0]<period[1]:
                    Cmd=[[1,3]]*len(Cmd)
                    if foot == 1 : #right stop 
                        motion[balance_step+(temp-1)*period[0]:excution_step] = motion_fr
                    else:
                        motion[balance_step+(temp-1)*period[0]:excution_step] = motion_fl
                else:
                    Cmd=[[0,3]]*len(Cmd)
                    if foot == 1 : #right stop 
                        motion[balance_step+(temp-1)*period[0]:excution_step] = motion_fr
                    else:
                        motion[balance_step+(temp-1)*period[0]:excution_step] = motion_fl
            elif (step>trans_point[0] and step<=trans_point[1]):
                temp = (step-trans_point[0])//period[1]+1
                foot = temp % 2
                excution_step=trans_point[0]+temp*period[1]
                if period[0]<period[1]:
                    Cmd=[[0,3]]*len(Cmd)
                    if foot == 1 : #right stop 
                        motion[trans_point[0]+(temp-1)*period[1]:excution_step] = motion_sr
                    else:
                        motion[trans_point[0]+(temp-1)*period[1]:excution_step] = motion_sl
                else:
                    Cmd=[[1,3]]*len(Cmd)
                    if foot == 1 : #right stop 
                        motion[trans_point[0]+(temp-1)*period[1]:excution_step] = motion_fr
                    else:
                        motion[trans_point[0]+(temp-1)*period[1]:excution_step] = motion_fl   
            elif (step>trans_point[1] and excution_step):
                temp = (step-trans_point[1])//period[2]+1
                foot = temp % 2
                excution_step=trans_point[1]+temp*period[2]
                if period[1]<period[2]:
                    Cmd=[[0,3]]*len(Cmd)
                    if foot == 1 : #right stop 
                        motion[trans_point[1]+(temp-1)*period[2]:excution_step] = motion_sr
                    else:
                        motion[trans_point[1]+(temp-1)*period[2]:excution_step] = motion_sl
                else:
                    Cmd=[[1,3]]*len(Cmd)
                    if foot == 1 : #right stop 
                        motion[trans_point[1]+(temp-1)*period[2]:excution_step] = motion_fr
                    else:
                        motion[trans_point[1]+(temp-1)*period[2]:excution_step] = motion_fl

        init_imu = IMU_data
        IMU_data = [a - b for a, b in zip(IMU_data, init_imu)] #校正
        if step<excution_step:
            if step == 25:
                a=input("go")
                time.sleep(1)

            if not reset:
                desired=Walk(step, motion,excution_step,[l_fsr,r_fsr],fsr_pos,CoP[-2:],IMU_data,Cmd,balance_step+2*period[0])

            for idx in range(len(init_pos_l)):
                desired[idx+2]-=init_pos_l[idx]

            for idx in range(len(init_pos_r)):
                desired[idx+8]-=init_pos_r[idx]
            send_cmd(desired)
            r_motion.append(pos)
            d_motion.append(list(desired))
            IMU.append(IMU_data)
            CoP.append(CoP_data)

        if step==excution_step-1 :
            print("plot")
            r_motion = np.array(r_motion)
            pd.DataFrame(r_motion).to_csv("Speed_ctrl/return.csv",header=None,index=False)
            pd.DataFrame(d_motion).to_csv("Speed_ctrl/desired.csv",header=None,index=False)
            pd.DataFrame(IMU).to_csv("Speed_ctrl/IMU.csv",header=None,index=False)
            pd.DataFrame(CoP).to_csv("Speed_ctrl/CoP.csv",header=None,index=False)
            msg1 = subprocess.Popen(['scp', 'Speed_ctrl/return.csv', 'Speed_ctrl/desired.csv', 'Speed_ctrl/src/F2.csv','Speed_ctrl/IMU.csv','Speed_ctrl/CoP.csv','airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Speed_ctrl'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            break
        
        if (abs(IMU[-1][1])>10 or distance<0.1):
            print(IMU[-1][1],distance)
            break 
        print(step)
        step+=1

if __name__ == '__main__':
    activation() # check connect / torque on 
    time.sleep(1)
    try:
        imu = threading.Thread(target = update_IMU)
        fsr = threading.Thread(target = update_fsr)
        cam = threading.Thread(target = update_realsense)
        recv = threading.Thread(target = get_status)
        cmd = threading.Thread(target = main)
        imu.start()
        fsr.start()
        cam.start()
        recv.start()
        cmd.start()
        imu.join()
        fsr.join()
        cam.join()
        recv.join()
        cmd.join()
        
        print("end")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        
