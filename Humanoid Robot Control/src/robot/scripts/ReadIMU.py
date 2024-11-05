#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import time
from IMU import *

def q_weight(queue):
    return queue[-4]*0.1+queue[-3]*0.2+queue[-2]*0.3+queue[-1]*0.4

def imu_q_update(imu_queue,imu,angle,standar):
    for i,q in enumerate(imu_queue):
        q.append(imu[i]-standar[i])
        q.pop(0)
        angle[i].append(q_weight(q))
    return imu_queue,angle

if __name__ == "__main__":
    xsens = Xsens(ShowError=False)     # initial the imu class

    # xsens.ConnectWithSerialNumber("DB5SGYLL")
    xsens.ConnectWithDeviceName("/dev/ttyUSB0")
    
    time.sleep(0.1)
    
    for _ in range(20):
        angle=[[],[]]
        rmax=0
        rmin=10
        pmax=0
        pmin=10
        step=0
        imu_queue=[[0]*4,[0]*4]
        standar=[0,0]
        while 1:
            xsens.GetMeasure()

            if xsens.NewDataAvailable() == True:
                xsens.MarkDataOld()
                xsens.QuatToEuler ()
                s = ""
                # s += str(time.time()) + " |Roll: %.2f" % (xsens.euler[0,0] * 180 / math.pi) + ", Pitch: %.2f" % (xsens.euler[0,1] * 180 / math.pi) + ", Yaw: %.2f " % (xsens.euler[0,2] * 180 / math.pi )
                if step != 0:
                    imu=[xsens.euler[0,0] * 180 / math.pi,xsens.euler[0,1] * 180 / math.pi]
                    if len(angle[0])!=0 and abs(xsens.euler[0,0]* 180 / math.pi - angle[0][-1]) > 10:
                        continue
                    imu_queue,angle=imu_q_update(imu_queue,imu,angle,standar)
                    if angle[0][-1]>rmax:
                        rmax=angle[0][-1]
                    if angle[0][-1]<rmin:
                        rmin = angle[0][-1]
                    if angle[1][-1]>pmax:
                        pmax=angle[1][-1]
                    if angle[1][-1]<pmin:
                        pmin = angle[1][-1]
                # imu_data.write("%s\n" % s)   #store imu_data
                    step+=1
                    # print(step,"Roll:",roll[-1],"Pitch:",pitch[-1])
                elif step == 0 and abs(xsens.euler[0,0]* 180 / math.pi)<1:
                    standar[0] = xsens.euler[0,0]* 180 / math.pi
                    standar[1] = xsens.euler[0,1]* 180 / math.pi
                    step+=1
                    # print(step,"Roll:",roll[-1],"Pitch:",pitch[-1])
                if step >500:
                    print("Roll:",rmax-rmin,"Pitch:",pmax-pmin)
                    break
                # print(step)
            else:
                # print("No data")
                pass
            time.sleep(0.005)
    x= np.linspace(1, step-1, step-1)
    plt.xlabel('Timestep (1 step = 0.02s)')
    plt.ylabel('Degree')
    plt.plot(x,angle[0],'r') 
    plt.plot(x,angle[1],'g') 

    plt.legend(['Roll','Pitch','Yaw'])
    plt.show()