import numpy as np
import pandas as pd
import csv
# 將弧度換算角度進行控制運算
d_motor=[]
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
        data_list[i].insert(0,float(0)) #加入trunk角度
    return data_list
def filt(step, desired, motion,i):
    modify = 0 # 19 is the base
    # if i == 2 and step>167+modify and step<176+modify and desired[i]< rad2deg(motion[167+modify][i]):
    #     desired[i]=rad2deg(motion[167+modify][i])

    if i == 2 and step>166+modify and step<211+modify:
        desired[i]=rad2deg(motion[166+modify][i])+1*(step-166+modify)*(rad2deg(motion[211+modify][i])-rad2deg(motion[166+modify][i]))/(211-166)
    # if i == 2 and step>235+modify and step<250+modify:
    #     desired[i]=rad2deg(motion[235+modify][i])+1*(step-235+modify)*(0-rad2deg(motion[235+modify][i]))/(250-235)
    # if i == 6 and step>235+modify and step<250+modify:
    #     desired[i]=rad2deg(motion[235+modify][i])+1*(step-235+modify)*(0-rad2deg(motion[235+modify][i]))/(250-235)
    # if i == 8 and step>235+modify and step<250+modify:
    #     desired[i]=rad2deg(motion[235+modify][i])+1*(step-235+modify)*(0-rad2deg(motion[235+modify][i]))/(250-235)
    if (i == 2 or i == 6 or i ==8 or i ==12) and step>220+modify and step<235+modify:
        desired[i]=rad2deg(motion[220+modify][i])+1*(step-220+modify)*(0-rad2deg(motion[220+modify][i]))/(235-220)
    if (i == 2 or i == 6 or i ==8 or i ==12) and step>234+modify:
        desired[i]=0

    if i == 9 and step > 170 and step < 250:
        desired[i]*=1.2
    # if i == 3 and step>140 and step<200 and desired[i]>rad2deg(motion[140][i]):
    #     desired[i]=rad2deg(motion[140][i])
    # elif i == 3 and step>200 and desired[i]>rad2deg(motion[len(motion)-1][i]):
    #     desired[i]=rad2deg(motion[len(motion)-1][i])
    # if i == 3 and step>198 and step<202:
    #     desired[i]=rad2deg(motion[200][i])
    if i == 4 and step>147 and step<190 and desired[i]<rad2deg(motion[147][i]):
        desired[i]=rad2deg(motion[147][i])
    # if i == 4 and step>200 and desired[i]<rad2deg(motion[len(motion)-1][i]):
    #     desired[i]=rad2deg(motion[len(motion)-1][i])
    # if i == 4 and step>140 and step<190 and desired[i]< rad2deg(motion[140][i]):
    #     desired[i]=rad2deg(motion[140][i])
    # if i == 5 and step>150 and step<200 and desired[i]>rad2deg(motion[154][i]):
    #     desired[i]=rad2deg(motion[154][i])
    # elif i == 5 and step>200 and desired[i]>rad2deg(motion[len(motion)-1][i]):
    #     desired[i]=rad2deg(motion[len(motion)-1][i])
    # elif i == 5 and step>192 and step<200:
    #     desired[i]=rad2deg(motion[196][i])
    # if i == 9 and step<90 and desired[i]>rad2deg(motion[0][i]):
    #     desired[i]=rad2deg(motion[0][i])
    # elif i == 9 and step>100 and step<151 and desired[i]>rad2deg(motion[len(motion)-1][i]):
    #     desired[i]=rad2deg(motion[len(motion)-1][i])
    # if i == 9 and step>95 and step<103:
    #     desired[i]=rad2deg(motion[99][i])
    # if i == 10 and desired[i]<rad2deg(motion[0][i]):
    #     desired[i]=40.15
    # if i == 10 and step>89 and step<107:
    #     desired[i]=rad2deg(motion[98][i])
    # elif i == 10 and step>125 and step<138:
    #     desired[i]=rad2deg(motion[126][i])
    # if i == 11 and step<90 and desired[i]>rad2deg(motion[0][i]):
    #     desired[i]=rad2deg(motion[0][i])
    # if i == 11 and step>92 and step<102:
    #     desired[i]=rad2deg(motion[97][i])
    return desired[i]

def Walk(step, desired, motion):
    global d_motor
    if step < len(motion):
        for i in range(len(motion[step])):
            desired[i] = rad2deg(motion[step][i])
            # desired[i] = filt(step, desired, motion,i)

    # if step<300:
    #     d_motor.append(list(desired[1:]))
    # if step==299:
    #     d_motor = np.array(d_motor)
    #     pd.DataFrame(d_motor).to_csv("Cho/desired.csv",header=None,index=False)
    return desired

# def CoP_compute(r_CoM,sensor_data):
#     x_cop = 0
#     y_cop = 0
#     rise = 0
#     # print(sensor_pos)
#     for idx,pos in enumerate(sensor_pos):
#        sensor_pos[idx]=pos-r_CoM
#     # print(sensor_pos)
#     #DSP 
#     if rise==0:
#        for i in range(len(sensor_pos)):
#             # print("DSP")
#             x_cop += sensor_data[i] * sensor_pos[i][0]
#             y_cop += sensor_data[i] * sensor_pos[i][1]
#     #SSP #Switch
#     elif rise==1 or rise==3:#右腳懸空/l2r
#        for i in range(int(len(sensor_pos)/2)):
#             # print("SSP L")
#             x_cop += sensor_data[i] * sensor_pos[i][0]
#             y_cop += sensor_data[i] * sensor_pos[i][1]
#     elif rise==2 or rise==4:#左腳懸空/r2l
#        for i in range(int(len(sensor_pos)/2)):
#             # print("SSP R")
#             x_cop += sensor_data[i+4] * sensor_pos[i+4][0]
#             y_cop += sensor_data[i+4] * sensor_pos[i+4][1]
#     x_cop /= (sum(sensor_data))
#     y_cop /= (sum(sensor_data))
#     # #做均值濾波
#     # x_cop = ((x_cop+r_CoM[0])+CoP_old[0][0]+CoP_old[1][0])/3
#     # y_cop = ((y_cop+r_CoM[1])+CoP_old[0][1]+CoP_old[1][1])/3
#     return np.array([x_cop,y_cop])

# def y_CoP_control(d_motion,rise,CoP_y,Rd_end_y,Ld_end_y,weight):
#     #DSP 實際
#     if rise==0 or rise==3 or rise==4:
#        if CoP_y<Rd_end_y : 
#           d_motion[2]+= d_motion[2]*weight[0]
#           d_motion[8]+=d_motion[8]*weight[0]
#           d_motion[6]+= d_motion[6]*weight[1]
#           d_motion[12]+=d_motion[12]*weight[1]
#        elif CoP_y>Ld_end_y : 
#           d_motion[2]-= d_motion[2]*weight[0]
#           d_motion[8]-=d_motion[8]*weight[0]
#           d_motion[6]-= d_motion[6]*weight[1] 
#           d_motion[12]-=d_motion[12]*weight[1]
#     #SSP
#     elif rise==1:#右腳懸空
#        if CoP_y>Ld_end_y : 
#           d_motion[2]+= d_motion[2]*weight[2]
#           d_motion[8]-=d_motion[8]*weight[2]
#           d_motion[6]-= d_motion[6]*weight[1] 
#           d_motion[12]-=d_motion[12]*weight[1]
#         #   d_motion[6]+= d_motion[6]*weight[1] #錯的
#         #   d_motion[12]+=d_motion[12]*weight[1]
#     elif rise==2:#左腳懸空
#        if CoP_y<Rd_end_y :
#           d_motion[2]-= d_motion[2]*weight[2]
#           d_motion[8]+=d_motion[8]*weight[2]
#           d_motion[6]+= d_motion[6]*weight[1]
#           d_motion[12]+=d_motion[12]*weight[1]
#         #   d_motion[6]-= d_motion[6]*weight[1]
#         #   d_motion[12]-=d_motion[12]*weight[1]
#     return d_motion[2],d_motion[6],d_motion[8],d_motion[12]
# def x_CoP_control(d_motion,rise,CoP_x,Rd_end_x,Ld_end_x,weight):
#     #DSP 
#     if rise==0:

#        if CoP_x>max(Rd_end_x,Ld_end_x): 
#           d_motion[5] = d_motion[5]*(1-weight[0])-d_motion[5]*weight[1]
#           d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[1]
#        elif CoP_x<min(Rd_end_x,Ld_end_x) : 
#           d_motion[5]= d_motion[5]*(1+weight[0])+d_motion[5]*weight[1]
#           d_motion[11]= d_motion[11]*(1+weight[0])+d_motion[11]*weight[1]
#     #SSP
#     elif rise==1:#右腳懸空
#        if CoP_x>max(Rd_end_x,Ld_end_x): 
#           d_motion[5]= d_motion[5]*(1-weight[0])-d_motion[5]*weight[3]
#         #   d_motion[11]+=d_motion[11]*weight[1] 
#        elif CoP_x<min(Rd_end_x,Ld_end_x) : 
#           d_motion[5]= d_motion[5]*(1+weight[0])+d_motion[5]*weight[3]
#         #   d_motion[11]-=d_motion[11]*weight[1]
#     elif rise==2:#左腳懸空
#        if CoP_x>max(Rd_end_x,Ld_end_x): 
#           d_motion[11]=d_motion[11]*(1-weight[0])-d_motion[11]*weight[3]
#         #   d_motion[5]+=d_motion[5]*weight[1]
#        elif CoP_x<min(Rd_end_x,Ld_end_x) : 
#           d_motion[11]=d_motion[11]*(1+weight[0])+d_motion[11]*weight[3]
#         #   d_motion[5]-=d_motion[5]*weight[1]
#     #Switch
#     elif rise==3:#l2r
#        if CoP_x>(Rd_end_x+Ld_end_x)/2 : 
#           d_motion[5]= d_motion[5]*(1+weight[0])+d_motion[5]*weight[2]
#           d_motion[11]= d_motion[11]*(1-weight[0])-d_motion[11]*weight[2]
#     elif rise==4:#r2l
#        if CoP_x>(Rd_end_x+Ld_end_x)/2 : 
#           d_motion[11]=d_motion[11]*(1+weight[0])+d_motion[11]*weight[2]
#           d_motion[5]= d_motion[5]*(1-weight[0])-d_motion[5]*weight[2]

#     return d_motion[5],d_motion[11]
