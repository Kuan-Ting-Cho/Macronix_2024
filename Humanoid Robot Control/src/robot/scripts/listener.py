#!/usr/bin/env python
import sys
import os
import rospy
from std_msgs.msg import Float32MultiArray
sys.path.append(os.path.abspath("/home/airobots/node/src"))
from MyActuator_motor.Myactuator import Myactuator
sys.path.append(os.path.abspath("/home/airobots/node/src/robot/scripts"))
from IMU import Xsens
import threading
import numpy as np
from ctypes import *
import time
from MotorUnion import MotorUnion
from math import pi
from fsr import FSR

id={11:0x14B,13:0x14D,14:0x14E,15:0x14F,21:0x155,23:0x157,24:0x158,25:0x159,1:0,12:1,16:2,22:3,26:4}
mode={0:'shut_down',1:'stop',2:'torque',3:'speed',4:'position',5:'pos_increment'
      ,6:'angle',7:'status',8:'reset',9:'torque_off',10:'torque_on',11:'motion_cmd'}
IMU_data=[0,0,0]
last_data = [0,0,0]
last_cmd=[0]*13
l_fsr=[0.0]*4
r_fsr=[0.0]*4
d_motion = []
r_motion = []
def callback(data):
    
    # recieve motor cmd
    cmd=np.array(data.data).reshape((13,7))
    # print(cmd)
    # cmd [id,mode,target value] MyActuator
    # if mode == 11 cmd [id,mode,pos, vel, t_ff, kp=10.012, kd=1]
    # a=time.time()
    for i in range(8):
        if cmd[i][1]!=11 or cmd[i][0]!=11 or cmd[i][0]!=21:
            motor.send_msg(id[cmd[i][0]], mode[cmd[i][1]], cmd[i][2])
            # motor.motion_cmd(id[cmd[i][0]],cmd[i][2])
            time.sleep(0.0015)
            # motor.send_msg(id[cmd[i][0]], mode[7],0)
            # time.sleep(0.001)
    # cmd [id,target value] dynamixal
    for i in range(8,13):
        if dy_mode == 1:
            dynamixal.SetMotor_Angle(id[cmd[i][0]],cmd[i][1])
            time.sleep(0.0015)
    # b=time.time()
    # print("cmd",b-a)

def reciever():
    rospy.Subscriber('command', Float32MultiArray, callback)
    while not rospy.is_shutdown():
        time.sleep(0.007) # 0.02-0.0005*13

def push_status():
    global IMU_data
    pub = rospy.Publisher('sensor', Float32MultiArray, queue_size=10)
    status=-np.ones((16,4),dtype='float32') #[11,13,14,15,21,23,24,25,1,12,16,22,26]
    key=list(id.keys())
    while not rospy.is_shutdown():
        # a=time.time()
        for i in range(8):
            if i == 0 or i == 4:
                continue
            tmp=motor.receive()
            while tmp[0] != key[i]:
                tmp=motor.receive()
            if abs(tmp[1])>360:
                tmp[1]=status[i][1]
            status[i][:]=tmp
            time.sleep(0.0001)
        for i in range(8,13):
            deg=dynamixal.GetPresentAngle(id[dy_id[i-8]])
            time.sleep(0.0005)
            spd=dynamixal.GetPresentVelocity(id[dy_id[i-8]])
            time.sleep(0.0005)
            cur=dynamixal.GetPresentCurrent(id[dy_id[i-8]])
            time.sleep(0.0005)
            status[i][:]=[dy_id[i-8],deg,spd,cur]
        status[-3][:3]=IMU_data
        status[-2]=l_fsr
        status[-1]=r_fsr
        
        # print(status)
        # add sensor data & motor status
        # print(status[-2])
        # print(status[-1])
        # print(sum(status[-2])+sum(status[-1]))
        state=Float32MultiArray(None,list(status.reshape(16*4)))
        pub.publish(state)
        time.sleep(0.0001)
        # b=time.time()
        # print("Recieve",b-a)

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
                last_data = IMU_data
                # print(xsens.euler[0,0]* 180 / pi - last_data[0],xsens.euler[0,1]* 180 / pi - last_data[0])
                if abs(xsens.euler[0,0]* 180 / pi - last_data[0]) > 20 or abs(xsens.euler[0,1]* 180 / pi - last_data[1]) > 20:
                    continue
                IMU_data=np.array([xsens.euler[0,0],xsens.euler[0,1],xsens.euler[0,2]])* 180 / pi
        time.sleep(0.005)

def update_fsr():
    global l_fsr
    global r_fsr
    left = FSR("/dev/ttyACM0", 9600, "l")
    right = FSR("/dev/ttyACM0", 9600, "r")
    while not rospy.is_shutdown():
        l_fsr = left.readData()
        r_fsr = right.readData()
        print(r_fsr)
        time.sleep(0.01)

if __name__ == '__main__':
    xsens = Xsens(ShowError=False)
    xsens.ConnectWithDeviceName("/dev/ttyUSB2")
    motor=Myactuator('/dev/ttyUSB1')
    dy_id=[1,12,16,22,26]
    dy_angle=[0.0,0.0,0.0,0.0,0.0]
    dy_mode=1
    dynamixal= MotorUnion()
    dynamixal.SetAllMotorsOperatingMode(dy_mode)
    dynamixal.SetAllMotorsTorqueEnableOn()
    
    bias=np.array([32,65,186.7,29.1,43,138.3,204.3,45.5,0,-1,0,1.5,0])# 11,13,14,15,21,23,24,25,1,12,16,22,26
    motor.send_msg(id[11], mode[4],bias[0])
    time.sleep(0.001)
    # motor.send_msg(id[13], mode[4], bias[1])
    # time.sleep(0.001)
    # motor.send_msg(id[14], mode[4], bias[2])
    # time.sleep(0.001)
    # motor.send_msg(id[15], mode[4], bias[3])
    # time.sleep(0.001)
    motor.send_msg(id[21], mode[4], bias[4])
    time.sleep(0.001)
    # motor.send_msg(id[23], mode[4], bias[5])
    # time.sleep(0.001)
    # motor.send_msg(id[24], mode[4], bias[6])
    # time.sleep(0.001)
    # motor.send_msg(id[25], mode[4], bias[7])
    # time.sleep(0.001)

    try:
        rospy.init_node('soldier', anonymous=True)
        sub = threading.Thread(target = reciever)
        pub = threading.Thread(target = push_status)
        imu = threading.Thread(target = update_IMU)
        fsr = threading.Thread(target = update_fsr)
        imu.start()
        sub.start()
        pub.start()
        # fsr.start()
        imu.join()
        sub.join()
        pub.join()
        # fsr.join()
        print("end")
    except rospy.ROSInterruptException:
        pass