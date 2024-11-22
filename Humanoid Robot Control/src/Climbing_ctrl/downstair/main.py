import sys
import os
sys.path.append(os.path.abspath('/home/airobots/node/src'))
from robot.scripts.cmd import *
from test import *
import threading
import numpy as np
from Cho.function import *
from Cho.lipm import *
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
    footstep=2
    lipm(footstep)
    # print("success")
    # send command using send_cmd, input:desired angle
    global pos
    d_motion = []
    r_motion = []
    step =0
    motion = csv2cmd('Cho/src/F2_filter.csv' )
    # print(motion)
    desired = [0.0]*13

    reset = 0
    init_pos_r=[0,0,0,0,0] #20 40
    init_pos_l=[0,0,0,0,0] #20 40
    # print(d_motion)
    while True:

        if step<300:
            desired=Walk(step, desired, motion)
            if reset:
                a=0.8
                desired=[0.0]*13
                init_pos_l=[0+a,rad2deg(0.35)+2,rad2deg(0.7)+2,rad2deg(0.35)+1.5,0-a] #22,40,20
                init_pos_r=[0+a,rad2deg(0.35)+2,rad2deg(0.7)+0,rad2deg(0.35)+5,0-a] #22,42,21
            
            desired[2]-=init_pos_l[0]
            desired[3]-=init_pos_l[1]
            desired[4]+=init_pos_l[2]
            desired[5]-=init_pos_l[3]
            desired[6]-=init_pos_l[4]

            desired[8]-=init_pos_r[0]
            desired[9]-=init_pos_r[1]
            desired[10]+=init_pos_r[2]
            desired[11]-=init_pos_r[3]
            desired[12]-=init_pos_r[4]
            # send_cmd(desired)
            
            r_motion.append(pos)
            d_motion.append(list(desired))
            # print(d_motion[-1])
            # print("\n")
        if step==299:
            print("plot")
            r_motion = np.array(r_motion)
            pd.DataFrame(r_motion).to_csv("Cho/return.csv",header=None,index=False)
            pd.DataFrame(d_motion).to_csv("Cho/desired.csv",header=None,index=False)
            msg1 = subprocess.Popen(['scp', 'Cho/return.csv', 'Cho/desired.csv', 'Cho/src/F2.csv','airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Cho'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            # msg2 = subprocess.Popen(['scp', 'Cho/desired.csv', 'airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Cho'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            # msg3 = subprocess.Popen(['scp', 'Cho/F2.csv', 'airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Cho'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            break
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
        
