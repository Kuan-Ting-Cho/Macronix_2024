import sys
import os
sys.path.append(os.path.abspath('/home/airobots/node/src'))
from robot.scripts.cmd import *
from test import *
import threading
import numpy as np
from Cho.function import *
from Cho.lipm_5 import *
from Cho.lipm_7 import *
from Cho.lipm_6 import *
from Cho.lipm_56 import *
import subprocess

status=np.ones((16,4),dtype='float32')
pos=[0.0]*13
l_fsr=[0.0]*4
r_fsr=[0.0]*4
IMU_data=[0,0,0]
def get_status():
    global status,pos,l_fsr,r_fsr,IMU_data
    while True:
        status,pos,_=update_status()
        IMU_data=status[-3][:3]
        l_fsr=status[-2]
        r_fsr=status[-1]
        
def main():
    # send command using send_cmd, input:desired angle
    global pos,IMU_data
    d_motion = []
    r_motion = []
    IMU = [[0.0,0.0,0.0]]
    step = 0

    Cmd="FS"
    total_step=12
    generate_step=8
    desired_one_step_time=[0.5,0.7]
    desired_step=[4,4,4]
    period = 50

    if Cmd != "N" and total_step!=desired_step[0]+desired_step[1]+desired_step[2]:
        print("step unequal!!!!!!  error!!!!!")
        time.sleep(100)

    reset = 0
    init_pos_l=[0]*5 #20 40
    init_pos_r=[0]*5 #20 40 
    init_roll=0
    # init_l=[init_roll, 0-1-1, 2.5-1, 3+2.5,-init_roll]  # hip+ 前踢/knee+ 伸直/ankle - 下壓
    # init_r=[init_roll+2, -3-1-1-1-1, 10.5-1+2, 9,-init_roll-1]
    init_l=[init_roll, 0+1, 2.5+1, 3,-init_roll]  # hip+ 前踢/knee+ 伸直/ankle - 下壓
    init_r=[init_roll, -3, 10.5, 9-2,-init_roll]
    if reset:
        init_pos_l=[0+init_l[0],rad2deg(0.3)+init_l[1],rad2deg(-0.6)+init_l[2],rad2deg(0.3)+init_l[3],0+init_l[4]] #22,40,20
        init_pos_r=[0+init_r[0],rad2deg(0.3)+init_r[1],rad2deg(-0.6)+init_r[2],rad2deg(0.3)+init_r[3],0+init_r[4]] #22,42,21
        # init_pos_l=[0]*5 #22,40,20
        # init_pos_r=[0]*5#22,42,21      
    if Cmd!="N":
        lipm_FSF(total_step,generate_step,desired_step,desired_one_step_time,init_l,init_r)
    elif period==60:
        lipm_6(total_step,generate_step,init_l,init_r)
        
    elif period==50:
        lipm_5(total_step,generate_step,init_l,init_r)
    elif  period==70:
        lipm_7(total_step,generate_step,init_l,init_r)

    motion = csv2cmd('Cho/src/F2_filter.csv' )
    motion1 = pd.read_csv('Cho/src/F2_filter.csv',header=None,index_col=None)
    # print(len(motion1.iloc[:,1]))

   
    Dynamix_delay = 7
    balance = 100
    # if total_step==generate_step:
    #     excution_step = 50 + total_step*period + Dynamix_delay + balance
    # else:
    #     excution_step = 50 + total_step*period + Dynamix_delay 

    forward_one_step = 50 + 50*(desired_step[0]+1) + Dynamix_delay 
    excution_step = len(motion)
    b=0
    while True:
        desired = [0.0]*13
        if step<excution_step:
            # print(step,IMU[-1][1])
            if step == 25:
                a=input("go")
                time.sleep(1)
            # if (not reset) and step == forward_one_step and b<desired_step[1] :
            #     forward_one_step += 50*1+10
            #     time.sleep(0.2)
            #     b+=1
                

            
            if not reset:
                desired=Walk(step, desired, motion,int(len(motion1.iloc[:,1])),period)

            for idx in range(len(init_pos_l)):
                desired[idx+2]-=init_pos_l[idx]

            for idx in range(len(init_pos_r)):
                desired[idx+8]-=init_pos_r[idx]
            send_cmd(desired)
            
            r_motion.append(pos)
            d_motion.append(list(desired))
            IMU.append(IMU_data)
            # print(d_motion[-1])
            # print("\n")
        if step==excution_step-1 :
        # if step==excution_step-1 or step==538:
            print("plot")
            r_motion = np.array(r_motion)

            pd.DataFrame(r_motion).to_csv("Cho/return.csv",header=None,index=False)
            pd.DataFrame(d_motion).to_csv("Cho/desired.csv",header=None,index=False)
            pd.DataFrame(IMU).to_csv("Cho/IMU.csv",header=None,index=False)
            msg1 = subprocess.Popen(['scp', 'Cho/return.csv', 'Cho/desired.csv', 'Cho/src/F2.csv','Cho/IMU.csv','airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Cho'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            break
        
        # if abs(IMU[-1][1]-5)>10:
        #     break
        print(step)
        step+=1

if __name__ == '__main__':
    activation()
    time.sleep(1)
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
        
