import sys
import os
sys.path.append(os.path.abspath('/home/airobots/node/src'))
from robot.scripts.cmd import *
from test import *
import threading
from balance import *
from lipm import *
from function import *
import time
import subprocess
from scipy.spatial.transform import Rotation as Rt
from forward_kinematics import *
from inverse_kinematics import *
status=np.ones((16,4),dtype='float32')
pos=[0.0]*13
l_fsr=[0.0]*4
r_fsr=[0.0]*4
IMU_data=[0,0,0]
step =0
qlen=4
imu_queue = [[0.0]*qlen,[0.0]*qlen,[0.0]*qlen]
angle_q=[[0.0]*10,[0.0]*10,[0.0]*10]
filter_imu=[]

def get_status():
    global status,pos,l_fsr,r_fsr,IMU_data,imu_queue,angle_q,filter_imu
    # standar = [-1000,-1000,0]
    while True:
        status,pos,_=update_status()
        IMU_data=status[-3][:3]
        l_fsr=status[-2]
        r_fsr=status[-1]
        # if standar[0]==-1000 and standar[1]==-1000:
        #     standar[0]=IMU_data[0]
        #     standar[1]=IMU_data[1]-1.5

        # imu_queue,angle_q=imu_q_update(imu_queue,IMU_data,angle_q,standar)
        # filter_imu=[angle_q[0][-1],angle_q[1][-1],angle_q[2][-1]]
        

def balance_controller(imu,sup_foot):
    rot_motor,P=split_m(forward_k(sup_foot))
    r = list((Rt.from_dcm(rot_motor)).as_euler("xyz"))
    r[0]+=deg2rad(imu[0])*0.01
    r[1]+=deg2rad(imu[1])*0.01
    print(imu[0]*0.01,imu[1]*0.01)
    rot_motor = Rt.from_euler('xyz',r).as_dcm()
    return InvK_CHI(P,rot_motor)

def split_m(matrix):
    R = [matrix[0][:3],matrix[1][:3],matrix[2][:3]]
    P = [matrix[0][-1],matrix[1][-1],matrix[2][-1]]
    return R,P

def main():
    # send command using send_cmd, input:desired angle
    global pos,IMU_data,l_fsr,r_fsr,step
    desired=[0.0]*13
    walk=0
    isbalance=1
    # time.sleep(3)
    if isbalance:
        while True:
            # print(IMU_data)
            # 只要做init pos的話,iszmp=1,isload=0,要做static impact的話要進去function解註解
            # 做負重平衡,iszmp=0,isload=1
            # 做板子平衡,iszmp=0,isload=0
            desired=balance_angle(desired,IMU_data,pos,l_fsr,r_fsr,iszmp=1,isload=0) 
            send_cmd(desired)
            # 負重平衡跟做initial pos需要限制step
            if step > 100:
                break
            step+=1
    time.sleep(1)

    # 走路 LIPM
    footstep = 2
    if walk:
        LIPM_obj = Forward2()
        LIPM_obj.output_motion(deg2rad(np.array(desired[1:])),footstep)
        motion = csv2cmd('Chaochi/src/F2_200.csv' )
        for _ in range(1):
            step=0
            desired=[0.0]*13
            r_motion=[]
            d_motion=[]
            motionstep=0
            while step < len(motion):

                desired=Walk(step, desired, motion)
                # a=time.time()
                # if step > 6 and step < 77:
                #     desired[1:7]=balance_controller(IMU_data,list(deg2rad(np.array(desired[1:7]))))
                #     # print("a",a)
                # elif step > 76 and step < 147:
                #     desired[7:]=balance_controller(IMU_data,list(deg2rad(np.array(desired[7:]))))
                    # print("b",b)
                # print(time.time()-a)
                r_motion.append(pos)
                d_motion.append(list(desired))
                send_cmd(desired)
                    # print(step)
                motionstep+=1
                if step==len(motion)-1:
                    print("plot")
                    r_motion = np.array(r_motion)
                    pd.DataFrame(r_motion).to_csv("Chaochi/return.csv",header=None,index=False)
                    pd.DataFrame(d_motion).to_csv("Chaochi/desired.csv",header=None,index=False)
                    msg1 = subprocess.Popen(['scp', 'Chaochi/return.csv', 'Chaochi/desired.csv', 'Chaochi/src/F2_origin.csv','airobots@192.168.1.232:~/Linkage_Robot_simulation/src/localcom/Chaochi'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    print("ok")
                step+=1
                # print(IMU_data)
            time.sleep(5)

if __name__ == '__main__':
    activation()
    try:
        imu = threading.Thread(target = update_IMU)
        fsr = threading.Thread(target = update_fsr)
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
        
