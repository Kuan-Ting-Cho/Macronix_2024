import numpy as np
import pandas as pd
import csv
from src.IVK import *
from src.DK import *
# 將弧度換算角度進行控制運算
def rad2deg(radius):
    return radius/np.pi*180
def csv2cmd(filename):
    file = open(filename)
    reader = csv.reader(file)
    data_list = list(reader)
    file.close()
    for i in range(len(data_list)):
        for j in range(len(data_list[i])):
            data_list[i][j] = float(data_list[i][j])
    return data_list

def Walk(step, motion,end,fsr_data,fsr_pos,pre_cop,imu,Cmd,no_ctrl):
    desired = [[0.0]*13,[0.0]*13,[0.0]*13] #[pre2,pre1,pre]
    if step<no_ctrl:
        for i in range(len(motion[0])):
            desired[2][i] = rad2deg(motion[step][i])
    elif step < len(motion):
        desired[2] = Controller(motion[step-2:step],fsr_data,fsr_pos,pre_cop,imu,Cmd[step-no_ctrl])
        for i in range(len(motion[step])):
            desired[2][i] = rad2deg(motion[step][i])
    else: #No Command
        for i in range(len(motion[0])):
            desired[2][i] = rad2deg(motion[end-1][i])
    return desired[2]

def Controller(desired,fsr_data,fsr_pos,pre_cop,imu_data,Cmd):
    ctrl=[0.0]*13
    parameter_sf = [0.03682629514048932,0.008146408284909503,0.0,0.0,0.0,4.2089995109205225e-05,0.025,0.025,0.010718184181988477,0.014073399487490331,0.01046094476405348,0.009015541855572285,0.011432635677570685,0.0]
    parameter_fs =[0.03682629514048932,0.008146408284909503,0.0,0.0,0.0,4.2089995109205225e-05,0.025,0.025,0.00011416779900600728,0.017891074895522607,0.014999052736591339,0.0036700688377963205,0.018189464614330544,0.012732542230948527]
    parameter = parameter_fs + parameter_sf[8:]
    R = euler_to_rotation_matrix(imu_data)
    Ld_end = DK(desired[2][1:7],R,2) #relative to base
    Rd_end = DK(desired[2][7:],R,2)
    phase = Sole_Status(fsr_data)
    CoP_x,CoP_y= CoP_compute(pre_cop,fsr_pos,fsr_data,phase)
    ctrl[5],ctrl[11],ctrl[3],ctrl[4],ctrl[9],ctrl[10] = x_CoP_control(desired[2],phase,CoP_x,Rd_end[0],Ld_end[0],parameter[4:8])
    ctrl[5],ctrl[11],ctrl[3],ctrl[4],ctrl[9],ctrl[10] = y_CoP_control(desired[2],desired,phase,CoP_y,Rd_end[1],Ld_end[1],parameter[0:4])
    ctrl[3],ctrl[4],ctrl[9],ctrl[10] = FTC_Ctrl(desired,phase,Rd_end,Ld_end,imu_data,Cmd,ctrl,parameter_fs[8:])
    return ctrl

def CoP_compute(CoP_old,fsr_pos,fsr,phase):
    sensordata = sum(fsr, []) #拉平
    x_cop = 0
    y_cop = 0
    #DSP 
    if phase==0:
       for i in range(len(fsr_pos)):
            x_cop += sensordata[i] * fsr_pos[i][0]
            y_cop += sensordata[i] * fsr_pos[i][1]
    #SSP #Switch
    elif phase==1 or phase==3:#右腳懸空/l2r
       for i in range(int(len(fsr_pos)/2)):
            x_cop += sensordata[i] * fsr_pos[i][0]
            y_cop += sensordata[i] * fsr_pos[i][1]
    elif phase==2 or phase==4:#左腳懸空/r2l
       for i in range(int(len(fsr_pos)/2)):
            x_cop += sensordata[i+4] * fsr_pos[i+4][0]
            y_cop += sensordata[i+4] * fsr_pos[i+4][1]
    x_cop /= (sum(sensordata))+0.000001
    y_cop /= (sum(sensordata))+0.000001
    #做均值濾波
    x_cop = (x_cop+CoP_old[0][0]+CoP_old[1][0])/3
    y_cop = (y_cop+CoP_old[0][1]+CoP_old[1][1])/3

    return x_cop,y_cop

def Sole_Status(fsr):
    sensordata = sum(fsr, []) #拉平
    #確認腳底板是否觸地(觸地即picth方向需平行)
    parallel=[]
    phase=0
    for idx in range(0,len(sensordata),2):
        if sensordata[idx]!=0 or sensordata[idx+1]!=0:
           parallel.append(1)
        else: 
           parallel.append(0)
    #確認腳是否懸空
    if  parallel[0]+parallel[1]== 0:
        phase = 2   #左腳懸空
    elif parallel[2]+parallel[3]== 0:
        phase = 1   #右腳懸空
    elif parallel[0]+parallel[1]+parallel[2]+parallel[3]==4:
        phase = 0   #DSP
    elif parallel[0]+parallel[1]+parallel[3]==3:
        phase = 3   #Switch l2r
    elif parallel[1]+parallel[2]+parallel[3]==3:
        phase = 4   #Switch r2l
    else:
        phase = 0   #DSP

    return phase
def y_CoP_control(d_motion,phase,CoP_y,Rd_end_y,Ld_end_y,weight):
    theta=0.05
    #DSP, SP
    if phase==0 or phase==3 or phase==4:
       if CoP_y<Rd_end_y-theta: 
          d_motion[2]+= d_motion[2]*weight[0]
          d_motion[8]+=d_motion[8]*weight[0]
          d_motion[6]+= d_motion[6]*weight[1]
          d_motion[12]+=d_motion[12]*weight[1]
       elif CoP_y>Ld_end_y+theta: 
          d_motion[2]-= d_motion[2]*weight[0] 
          d_motion[8]-=d_motion[8]*weight[0]
          d_motion[6]-= d_motion[6]*weight[1] 
          d_motion[12]-=d_motion[12]*weight[1]
    #SSP
    elif phase==1:#右腳懸空
       if CoP_y>Ld_end_y+theta: 
          d_motion[2]-= d_motion[2]*weight[2]
          d_motion[6]-= d_motion[6]*weight[3] 

    elif phase==2:#左腳懸空
       if CoP_y<Rd_end_y-theta :
          d_motion[8]+= d_motion[8]*weight[2]
          d_motion[12]+= d_motion[12]*weight[3]

    return d_motion[2],d_motion[6],d_motion[8],d_motion[12]

def x_CoP_control(d_motion,phase,CoP_x,Rd_end_x,Ld_end_x,weight):
    theta=0.05
    #DSP 
    if phase==0:
       if CoP_x>max(Rd_end_x,Ld_end_x)+theta: 
          d_motion[3]= d_motion[3]*(1-weight[0])-d_motion[3]*weight[1]
          d_motion[4]= d_motion[4]*(1+weight[0])+d_motion[4]*weight[1]
          d_motion[5] = d_motion[5]*(1-weight[0])-d_motion[5]*weight[1]

          d_motion[9]= d_motion[9]*(1-weight[0])-d_motion[9]*weight[1]
          d_motion[10]= d_motion[10]*(1+weight[0])+d_motion[10]*weight[1]
          d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[1]

       elif CoP_x<min(Rd_end_x,Ld_end_x)-theta : 
          d_motion[3]= d_motion[3]*(1+weight[0])+d_motion[3]*weight[1]
          d_motion[4]= d_motion[4]*(1-weight[0])-d_motion[4]*weight[1]
          d_motion[5] = d_motion[5]*(1+weight[0])+d_motion[5]*weight[1]

          d_motion[9]= d_motion[9]*(1+weight[0])+d_motion[9]*weight[1]
          d_motion[10]= d_motion[10]*(1-weight[0])-d_motion[10]*weight[1]
          d_motion[11]= d_motion[11]*(1+weight[0])+d_motion[11]*weight[1]
    #SSP
    elif phase==1:#右腳懸空
       if CoP_x>Ld_end_x+theta: 
          d_motion[3]= d_motion[3]*(1-weight[0])-d_motion[3]*weight[3]
          d_motion[4]= d_motion[4]*(1+weight[0])+d_motion[4]*weight[3]
          d_motion[5] = d_motion[5]*(1-weight[0])-d_motion[5]*weight[3]

       elif CoP_x<Ld_end_x-theta : 
          d_motion[3]= d_motion[3]*(1+weight[0])+d_motion[3]*weight[3]
          d_motion[4]= d_motion[4]*(1-weight[0])-d_motion[4]*weight[3]
          d_motion[5]= d_motion[5]*(1+weight[0])+d_motion[5]*weight[3]

    elif phase==2:#左腳懸空
       if CoP_x>Rd_end_x+theta: 
          d_motion[9]= d_motion[9]*(1-weight[0])-d_motion[9]*weight[3]
          d_motion[10]= d_motion[10]*(1+weight[0])+d_motion[10]*weight[3]
          d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[3]

       elif CoP_x<Rd_end_x-theta: 
          d_motion[9]= d_motion[9]*(1+weight[0])+d_motion[9]*weight[3]
          d_motion[10]= d_motion[10]*(1-weight[0])-d_motion[10]*weight[3]
          d_motion[11]= d_motion[11]*(1+weight[0])+d_motion[11]*weight[3]
    #SP
    elif phase==3:#l2r
        if CoP_x>(Rd_end_x+Ld_end_x)/2 : 
            d_motion[3]= d_motion[3]*(1+weight[0])+d_motion[3]*weight[2]
            d_motion[4]= d_motion[4]*(1-weight[0])-d_motion[4]*weight[2]
            d_motion[5]= d_motion[5]*(1+weight[0])+d_motion[5]*weight[2]

            d_motion[9]= d_motion[9]*(1-weight[0])-d_motion[9]*weight[2]
            d_motion[10]= d_motion[10]*(1+weight[0])+d_motion[10]*weight[2]
            d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[2]

    elif phase==4:#r2l
       if CoP_x>(Rd_end_x+Ld_end_x)/2 : 
            d_motion[9]= d_motion[9]*(1+weight[0])+d_motion[9]*weight[2]
            d_motion[10]= d_motion[10]*(1-weight[0])-d_motion[10]*weight[2]
            d_motion[11]= d_motion[11]*(1+weight[0])+d_motion[11]*weight[2]

            d_motion[3]= d_motion[3]*(1-weight[0])-d_motion[3]*weight[2]
            d_motion[4]= d_motion[4]*(1+weight[0])+d_motion[4]*weight[2]
            d_motion[5] = d_motion[5]*(1-weight[0])-d_motion[5]*weight[2]

    return d_motion[5],d_motion[11],d_motion[3],d_motion[4],d_motion[9],d_motion[10]

#################### Foot Trajactory Control ####################
def FTC_Ctrl(d_motion,phase,Rd_end,Ld_end,R,euler_angle,Cmd,CoP_ctrl,weight): 
    Rd_end[1]=(Rd_end[1]-0.105) # relative to R_yaw
    Ld_end[1]=(Ld_end[1]+0.105) # relative to L_yaw
    pitch=euler_angle[1]
    range=3
    if Cmd==[1,2] : 
        value_z=weight[0]
        value_x=weight[0]
        value=weight[1]
    elif Cmd==[0,2] or Cmd==[0,3]: 
        value_z=weight[2] 
        value_x=weight[2]
        value=weight[3]
    elif Cmd==[1,0] :   
        value_z=weight[4]
        value_x=weight[4]
        value=weight[5]
    elif Cmd==[0,1] :
        value_z=weight[6]
        value_x=weight[6]
        value=weight[7]
    elif Cmd==[1,1] or Cmd==[1,3] :
        value_z=weight[8]
        value_x=weight[8]
        value=weight[9]
    elif Cmd==[0,0] :
        value_z=weight[10]
        value_x=weight[10]
        value=weight[11]
    else :
        print("out of control")
    #value_z蹲為正/起為負
    #value_x蹲為負/起為正
    if pitch<-range:
        change=True
        if phase==1 or phase==3: #右腳離地/l2r
            PR=Rd_end+[0,0,-value_z]
            PL=Ld_end+[+value_x,0,-value_z]
        elif phase==2 or phase==4: #左腳離地/r2l
            PL=Rd_end+[0,0,-value_z]
            PR=Ld_end+[+value_x,0,-value_z]
        elif phase==0: 
            PR=Rd_end+[0,0,-value] 
            PL=Ld_end+[0,0,-value]
    elif pitch>range:
        change=True
        if phase==1 or phase==3: #右腳離地/l2r
            PR=Rd_end+[0,0,+value_z]
            PL=Ld_end+[-value_x,0,+value_z]
        elif phase==2 or phase==4: #左腳離地/r2l
            PL=Rd_end+[0,0,+value_z]
            PR=Ld_end+[-value_x,0,+value_z]
        elif phase==0: 
            PR=Rd_end+[0,0,+value] 
            PL=Ld_end+[0,0,+value]
    if change:
        d_motion.iloc[2,3:5]+=InvK(PL,R)[2:4]-d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]+=InvK(PR,R)[2:4]-d_motion.iloc[2,9:11]
        d_motion.iloc[2,3:5]=0.3*d_motion.iloc[0,3:5]+0.3*d_motion.iloc[1,3:5]+0.4*d_motion.iloc[2,3:5]
        d_motion.iloc[2,9:11]=0.3*d_motion.iloc[0,9:11]+0.3*d_motion.iloc[1,9:11]+0.4*d_motion.iloc[2,9:11]  
        
    else:
        d_motion.iloc[2,3]=CoP_ctrl[0]
        d_motion.iloc[2,4]=CoP_ctrl[1]
        d_motion.iloc[2,9]=CoP_ctrl[2]
        d_motion.iloc[2,10]=CoP_ctrl[3]

    return d_motion.iloc[2,3],d_motion.iloc[2,4],d_motion.iloc[2,9],d_motion.iloc[2,10]

def euler_to_rotation_matrix(imu_data):
    [roll, pitch, yaw]=imu_data
    c_yaw, s_yaw = np.cos(yaw), np.sin(yaw)
    c_pitch, s_pitch = np.cos(pitch), np.sin(pitch)
    c_roll, s_roll = np.cos(roll), np.sin(roll)

    R = np.array([
        [c_yaw * c_pitch, c_yaw * s_pitch * s_roll - s_yaw * c_roll, c_yaw * s_pitch * c_roll + s_yaw * s_roll],
        [s_yaw * c_pitch, s_yaw * s_pitch * s_roll + c_yaw * c_roll, s_yaw * s_pitch * c_roll - c_yaw * s_roll],
        [-s_pitch,         c_pitch * s_roll,                          c_pitch * c_roll]
    ])
    return R
a=0
Controller(a)