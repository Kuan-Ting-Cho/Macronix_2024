from ctypes import *
import time
import numpy as np
lib = cdll.LoadLibrary('./MotorUnion/build/libMotorUnion.so')
class MotorUnion(object):
    def __init__(self):
        self.obj = lib.MotorUnion_new()
    def MotorID2idx(self,id):
        ID_list = [12, 16]
        for idx,ID in enumerate(ID_list):
            if ID==id:
                return idx
                break

    # Set data #
    def SetAllMotorsOperatingMode(self,mode):
        lib.SetAllMotorsOperatingMode_new(self.obj,mode)
    def SetAllMotorsTorqueEnableOn(self):
        lib.SetAllMotorsTorqueEnableOn_new(self.obj)
        time.sleep(0.01) 
    def SetAllMotorsTorqueEnableOff(self):
        lib.SetAllMotorsTorqueEnableOff_new(self.obj)
        time.sleep(0.01) 
    def Set_Velocity(self,id,vel):
        lib.SetMotor_Velocity_new(self.obj,c_ubyte(id),c_float(vel))
    def Set_Angle(self,id,angle):
        lib.SetMotor_Angle_new(self.obj,c_ubyte(id),c_float(angle))
    def Set_Current(self,id,current):
        lib.SetMotor_Current_new(self.obj,c_ubyte(id),c_float(current))

    def Current_Control(self,id_list,vel_list): 
        for i in range(len(id_list)):
            self.Set_Current(self.MotorID2idx(id_list[i]),vel_list[i])
    def Velocity_Control(self,id_list,vel_list): 
        for i in range(len(id_list)):
            self.Set_Velocity(self.MotorID2idx(id_list[i]),vel_list[i])
    def Postion_Control(self,id_list,angle_list): 
        for i in range(len(id_list)):
            self.Set_Angle(self.MotorID2idx(id_list[i]),angle_list[i])
    # Get data #
    def Get_ID(self,id):
        return lib.GetMotor_ID_new(self.obj,id)
    def Get_TorqueEnable(self,id):
        return lib.GetMotor_TorqueEnable_new(self.obj,id)
    def Get_Operating_Mode(self,id):
        return lib.GetMotor_Operating_Mode_new(self.obj,id)
    def Get_PresentAngle(self,id):
        lib.GetMotor_PresentAngle_new.restype=c_float # 設定函數返回型別為 float
        return lib.GetMotor_PresentAngle_new(self.obj,id)
    def Get_PresentVelocity(self,id):
        lib.GetMotor_PresentVelocity_new.restype=c_float
        return lib.GetMotor_PresentVelocity_new(self.obj,id)
    def Get_PresentCurrent(self,id):
        lib.GetMotor_PresentCurrent_new.restype=c_float
        return lib.GetMotor_PresentCurrent_new(self.obj,id)
    def Get_Angle(self,id):
        lib.GetMotor_Angle_new.restype=c_float
        return lib.GetMotor_Angle_new(self.obj,id)
    def Get_Velocity(self,id):
        lib.GetMotor_Velocity_new.restype=c_float
        return lib.GetMotor_Velocity_new(self.obj,id)
    def Get_Current(self,id):
        lib.GetMotor_Current_new.restype=c_float
        return lib.GetMotor_Current_new(self.obj,id)
    
    # Get data form #
    def Motor_data(self):
        data=[]
        for i in range(2):
            motor_data=[]
            motor_data.append(self.Get_ID(i))
            mode=self.Get_Operating_Mode(i)
            if mode==0:
               sign=np.sign(self.Get_Current(i))
            elif mode==1:
               sign=np.sign(self.Get_Velocity(i))
            elif mode==3:
               sign=np.sign(self.Get_Angle(i))
            motor_data.append(sign*round(self.Get_PresentAngle(i)%360, 2)) #degree
            motor_data.append(round(self.Get_PresentVelocity(i)*(6/100), 2)) #degree/s

            if i==0 or i==2 or i==4: #pro20
               y=abs(self.Get_PresentCurrent(i))/1000
               torque=sign*(y-0.267)/0.222
            else: #pro100
               y=abs(self.Get_PresentCurrent(i))/1000
               torque=sign*(y-0.907)/0.183
            motor_data.append(round(torque, 2)) #torque
            motor_data.append(self.Get_PresentCurrent(i)/1000) #mA
            data.append(motor_data)
        return data
    
mode=0
id= [12,16]
vel=[-4000.0,-4000.0]   #float
vel=[0.0,0.0]   #float
angle=[-100.0,-100.0] #float
current=[-3000.0,-3000.0] #float
current=[0.0,0.0] #float
Motor= MotorUnion()
Motor.SetAllMotorsOperatingMode(mode)

Motor.SetAllMotorsTorqueEnableOn()
# Motor.Postion_Control(id,angle)
# Motor.Velocity_Control(id,vel)
Motor.Current_Control(id,current)
time.sleep(0.02)
data=Motor.Motor_data()
print(data)
i=0
while True:
    print(data,i)
    data=Motor.Motor_data()
    time.sleep(1)
    i+=1
# Motor.SetAllMotorsTorqueEnableOff()