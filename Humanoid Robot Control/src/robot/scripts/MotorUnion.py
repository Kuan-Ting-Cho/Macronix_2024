from ctypes import *
import time

lib = cdll.LoadLibrary('Motor_Driver/src/MotorUnion/build/libMotorUnion.so')

class MotorUnion(object):
    def __init__(self):
        self.obj = lib.MotorUnion_new()
    def MotorID2idx(self,id):
        ID_list = [1,12, 16,22,26]
        for idx,ID in enumerate(ID_list):
            if ID==id:
                return idx
    # Set data #
    def SetAllMotorsOperatingMode(self,mode):
        lib.SetAllMotorsOperatingMode_new(self.obj,mode)
    def SetAllMotorsTorqueEnableOn(self):
        lib.SetAllMotorsTorqueEnableOn_new(self.obj)
        time.sleep(0.01) 
    def SetAllMotorsTorqueEnableOff(self):
        lib.SetAllMotorsTorqueEnableOff_new(self.obj)
        time.sleep(0.01) 
    def SetMotor_Velocity(self,id,vel):
        lib.SetMotor_Velocity_new(self.obj,c_ubyte(id),c_int(vel))
    def SetMotor_Angle(self,id,angle):
        lib.SetMotor_Angle_new(self.obj,c_ubyte(id),c_float(angle))

    def Velocity_control(self,id_list,vel_list): 
        for i in range(len(id_list)):
            self.SetMotor_Velocity(self.MotorID2idx(id_list[i]),vel_list[i])
    def Position_control(self,id_list,angle_list): 
        for i in range(len(id_list)):
            self.SetMotor_Angle(self.MotorID2idx(id_list[i]),angle_list[i])
        # time.sleep(0.03)
    # Get data #
    def GetID(self,id):
        return lib.GetMotor_ID_new(self.obj,id)
    def GetTorqueEnable(self,id):
        return lib.GetMotor_TorqueEnable_new(self.obj,id)
    def GetOperating_Mode(self,id):
        return lib.GetMotor_Operating_Mode_new(self.obj,id)
    def GetPresentAngle(self,id):
        lib.GetMotor_PresentAngle_new.restype=c_float # 設定函數返回型別為 float
        return lib.GetMotor_PresentAngle_new(self.obj,id)
    def GetPresentVelocity(self,id):
        lib.GetMotor_PresentVelocity_new.restype=c_float
        return lib.GetMotor_PresentVelocity_new(self.obj,id)
    def GetPresentCurrent(self,id):
        lib.GetMotor_PresentCurrent_new.restype=c_float
        return lib.GetMotor_PresentCurrent_new(self.obj,id)
    def GetAngle(self,id):
        lib.GetMotor_Angle_new.restype=c_float
        return lib.GetMotor_Angle_new(self.obj,id)
    def GetVelocity(self,id):
        return lib.GetMotor_Velocity_new(self.obj,id)
    def GetAccel(self,id):
        return lib.GetMotor_Accel_new(self.obj,id)
    
    # Get data form #
    def Motor_data(self):
        data=[]
        for i in range(2):
            motor_data=[]
            motor_data.append(self.GetID(i))
            motor_data.append(self.GetTorqueEnable(i))
            motor_data.append(self.GetOperating_Mode(i))
            motor_data.append(round(self.GetPresentAngle(i), 2))
            motor_data.append(round(self.GetPresentVelocity(i), 2))
            motor_data.append(round(self.GetPresentCurrent(i), 2))
            motor_data.append(round(self.GetAngle(i), 2))
            motor_data.append(self.GetVelocity(i))
            motor_data.append(self.GetAccel(i))
            data.append(motor_data)
        return data