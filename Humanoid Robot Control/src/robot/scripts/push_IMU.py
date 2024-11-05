import os
import sys
import rospy
from std_msgs.msg import Float32MultiArray
sys.path.append(os.path.abspath("/home/airobots/node/src/robot/scripts"))
from IMU import Xsens
import threading
import numpy as np
import time
from math import pi

IMU_data=[0,0,0]

def push_imudata():
    while not rospy.is_shutdown():
        pub = rospy.Publisher('IMU', Float32MultiArray, queue_size=10)
        state=Float32MultiArray(None,list(IMU_data))
        pub.publish(state)
        print(IMU_data)
        time.sleep(0.05)

def update_IMU():
    global IMU_data
    while not rospy.is_shutdown():
        xsens.GetMeasure()
        if xsens.NewDataAvailable() == True:
            xsens.MarkDataOld()
            xsens.QuatToEuler()
            if xsens.euler[0,0]==-1 and xsens.euler[0,1]==-1 and xsens.euler[0,2]==-1:
                print("Error")
            else:
                IMU_data=np.array([xsens.euler[0,0],xsens.euler[0,1],xsens.euler[0,2]])* 180 / pi
        time.sleep(0.05)

if __name__ == "__main__":
    xsens = Xsens(ShowError=False)
    xsens.ConnectWithDeviceName("/dev/ttyUSB1")
    try:
        rospy.init_node('IMUDATA', anonymous=True)
        imu = threading.Thread(target = update_IMU)
        imu.start()
        push_imudata()
        imu.join()
        print("end")
    except rospy.ROSInterruptException:
        pass