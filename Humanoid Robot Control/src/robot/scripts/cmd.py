import sys
import os
sys.path.append(os.path.abspath("/home/airobots/node/src"))
from MyActuator_motor.Myactuator import Myactuator
sys.path.append(os.path.abspath("/home/airobots/node/src/robot/scripts"))
from IMU import Xsens
import numpy as np
from ctypes import *
import time
from MotorUnion import MotorUnion
from realsense import realsense
from math import pi
from fsr import FSR

index={11:0,13:1,14:2,15:3,21:4,23:5
      ,24:6,25:7,1:8,12:9,16:10,22:11,26:12}
bias=np.array([33-2,141.5,191.5,33.6,41-4,65,186.3,160,0,-1,0,1.5,0])# 11,13,14,15,21,23,24,25,1,12,16,22,26
limit=np.array([[51,-7],[140,55],[104.5,10],[76,-0.6],[80,22],[26,-66],[25,-70],[91,1.6]]) #要寫極限角判斷
pos=[0.0]*13 # 對應desired
id={11:0x14B,13:0x14D,14:0x14E,15:0x14F,21:0x155,23:0x157,24:0x158,25:0x159,1:0,12:1,16:2,22:3,26:4}
mode={0:'shut_down',1:'stop',2:'torque',3:'speed',4:'position',5:'pos_increment'
      ,6:'angle',7:'status',8:'reset',9:'torque_off',10:'torque_on',11:'motion_cmd'}
IMU=[0,0,0]
last_data = [0,0,0]
last_cmd=[0]*13
left_fsr=[0.0]*4
right_fsr=[0.0]*4
distance = 0.0
# motor,sensor
my_motor=0
xsens=0
dynamixal=0
dy_id=[1,12,16,22,26]
dy_angle=[0.0,0.0,0.0,0.0,0.0]
dy_mode=1

def send_cmd(desired):
    global my_motor,dynamixal,dy_mode
    motor=-np.ones((13,7))
    motor[index[11]][:3]=np.array([11,4,bias[0]],dtype=np.float64)
    motor[index[13]][:3]=np.array([13,4,-desired[9]+bias[1]],dtype=np.float64)
    motor[index[14]][:3]=np.array([14,4,desired[10]+bias[2]],dtype=np.float64)
    motor[index[15]][:3]=np.array([15,4,-desired[11]+bias[3]],dtype=np.float64)
    motor[index[21]][:3]=np.array([21,4,bias[4]],dtype=np.float64)
    motor[index[23]][:3]=np.array([23,4,desired[3]+bias[5]],dtype=np.float64)
    motor[index[24]][:3]=np.array([24,4,-desired[4]+bias[6]],dtype=np.float64)
    motor[index[25]][:3]=np.array([25,4,desired[5]+bias[7]],dtype=np.float64)
    motor[index[1]][:2]=np.array([1,desired[0]+bias[8]],dtype=np.float64)
    motor[index[12]][:2]=np.array([12,desired[8]+bias[9]],dtype=np.float64)
    motor[index[16]][:2]=np.array([16,desired[12]+bias[10]],dtype=np.float64)
    motor[index[22]][:2]=np.array([22,desired[2]+bias[11]],dtype=np.float64)
    motor[index[26]][:2]=np.array([26,desired[6]+bias[12]],dtype=np.float64)
    
    for i in range(8):
        if motor[i][1]!=11 or motor[i][0]!=11 or motor[i][0]!=21:
            my_motor.send_msg(id[motor[i][0]], mode[motor[i][1]], motor[i][2])
            # motor.motion_cmd(id[cmd[i][0]],cmd[i][2])
            time.sleep(0.0015)
            # motor.send_msg(id[cmd[i][0]], mode[7],0)
    # cmd [id,target value] dynamixal
    for i in range(8,13):
        if dy_mode == 1:
            dynamixal.SetMotor_Angle(id[motor[i][0]],motor[i][1])
            time.sleep(0.0015)
    
    return 1

def update_IMU():
    global IMU,xsens
    while True:
        xsens.GetMeasure()
        if xsens.NewDataAvailable() == True:
            xsens.MarkDataOld()
            xsens.QuatToEuler()
            if xsens.euler[0,0]==-1 and xsens.euler[0,1]==-1 and xsens.euler[0,2]==-1:
                print("Error")
            else:
                last_data = IMU
                # print(xsens.euler[0,0]* 180 / pi - last_data[0],xsens.euler[0,1]* 180 / pi - last_data[0])
                if abs(xsens.euler[0,0]* 180 / pi - last_data[0]) > 20 or abs(xsens.euler[0,1]* 180 / pi - last_data[1]) > 20:
                    continue
                IMU=np.array([xsens.euler[0,0],xsens.euler[0,1],xsens.euler[0,2]])* 180 / pi
        time.sleep(0.005)
        # print(IMU)
        

def update_fsr():
    global left_fsr
    global right_fsr
    right = FSR("/dev/ttyACM0", 9600, "l")
    left = FSR("/dev/ttyACM1", 9600, "r")
    while True:
        left_fsr = left.readData()
        right_fsr = right.readData()
        # print("left: ",left_fsr)
        # print("right: ",right_fsr)
        time.sleep(0.01)
def update_realsense():
    global distance
    camera = realsense()  # 創建 realsense 
    while True:
        try:
            frames = camera.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            dist_sum = 0.0
            for y in range(50, 200):
                for x in range(100, 540):
                    dist_sum += depth_frame.get_distance(x, y)
            distance = dist_sum / 66000  

        except Exception as e:
            print(f"Error in update_realsense: {e}")
        time.sleep(0.01)


def update_status():
    global IMU,distance,pos,left_fsr,right_fsr,my_motor,dynamixal,dy_id
    status=-np.ones((17,4),dtype='float32') #[11,13,14,15,21,23,24,25,1,12,16,22,26]
    key=list(id.keys())
    a=time.time()
    for i in range(8):
        if i == 0 or i == 4:
            continue
        tmp=my_motor.receive()
        while tmp[0] != key[i]:
            tmp=my_motor.receive()
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
    status[-4][:1]=distance
    status[-3][:3]=IMU
    status[-2]=left_fsr
    status[-1]=right_fsr
    pos = [status[index[1]][1]-bias[index[1]],status[index[21]][1]-bias[index[21]],status[index[22]][1]-bias[index[22]],
        status[index[23]][1]-bias[index[23]],-(status[index[24]][1]-bias[index[24]]),status[index[25]][1]-bias[index[25]],
        status[index[26]][1]-bias[index[26]],status[index[11]][1]-bias[index[11]],status[index[12]][1]-bias[index[12]],
        -(status[index[13]][1]-bias[index[13]]),status[index[14]][1]-bias[index[14]],-(status[index[15]][1]-bias[index[15]]),
        status[index[16]][1]-bias[index[16]]]
    vel = [status[index[1]][2],status[index[21]][2],status[index[22]][2],
        status[index[23]][2],-(status[index[24]][2]),status[index[25]][2],
        status[index[26]][2],status[index[11]][2],status[index[12]][2],
        -(status[index[13]][2]),status[index[14]][2],-(status[index[15]][2]),
        status[index[16]][2]]
    return status,pos,vel

def activation():
    global my_motor,xsens,dynamixal,dy_id,bias
    xsens = Xsens(ShowError=False)
    xsens.ConnectWithDeviceName("/dev/ttyUSB2")
    my_motor=Myactuator('/dev/ttyUSB1')
    dynamixal= MotorUnion()
    dynamixal.SetAllMotorsOperatingMode(dy_mode)
    dynamixal.SetAllMotorsTorqueEnableOn()
    # bias=np.array([32,126.7,186.7,29.1,50,138.3,204.3,45.5,0,-1,0,1.5,0])# 11,13,14,15,21,23,24,25,1,12,16,22,26
    my_motor.send_msg(id[11], mode[4],bias[0])
    time.sleep(0.001)
    # my_motor.send_msg(id[13], mode[4],bias[1])
    # time.sleep(0.001)
    # my_motor.send_msg(id[14], mode[4],bias[2])
    # time.sleep(0.001)
    # my_motor.send_msg(id[15], mode[4],bias[3])
    # time.sleep(0.001)
    my_motor.send_msg(id[21], mode[4], bias[4])
    time.sleep(0.001)
    # my_motor.send_msg(id[23], mode[4], bias[5])
    # time.sleep(0.001)
    # print("ok")
    # my_motor.send_msg(id[24], mode[4], bias[6])
    # time.sleep(0.001)
    # print("ok")
    # my_motor.send_msg(id[25], mode[4], bias[7])
    # time.sleep(0.001)
    # print("ok")
