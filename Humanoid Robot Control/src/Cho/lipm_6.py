from src.LIPMMotionGenerator import LIPM_motion_generator
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataprocess import Data_preprocess
import time
def deg2rad(angle):
    # global np.pi
    return angle*np.pi/180
def lipm_6(total_step,generate_step,init_l,init_r):
    # step = 2.2
    print("run")
    """ Common Setting """
    fileName = 'F2'    # outputName
    footStep = generate_step
    motionGen = True
    cmdGen = False
    unit = 0 # 0 => for simulation; 1 => for real world
    """ Define type of motion """
    rightFirst = True
    forward = True
    shift = False
    turn = False
    """ Setting of LIPM parameters """
    # Amplitude of swing
    b1 = -0.400   #+0.025  #左腳重心外側 -0.165-0.01
    b2 = 0.400 #-0.037 #+0.02    -0.037  46
    b3 = -0.400
    b4 = 0.400
    b5 = -0.400
    b6 = 0.400
    # Motor 31/35 and Motor 41/45
    # 正： 腰與腳踝彎曲方向一致
    hip1 = 0.5   #-0.1
    hip2 = 0.3 #-0.4
    # Step Height
    stepHeight1 = 0.06  #0.15
    stepHeight2 = 0.06  #0.17
    stepHeight3 = 0.06
    stepHeight4 = 0.06
    stepHeight5 = 0.06
    stepHeight6 = 0.06
    # Forward Distance
    stepSize1 = 0.07
    stepSize2 = 0.05
    stepSize3 = 0.05
    stepSize4 = 0.06
    stepSize5 = 0.05
    stepSize6 = 0.0
    # Lateral Displacement
    shift1 = np.array([deg2rad(0)]*footStep)
    shift2 = np.array([deg2rad(0)]*footStep)

    # Motor 30 and Motor 40
    yawAngleR = [[0, 0, 0, 0, 0]]*footStep
    yawAngleL = [[0, 0, 0, 0, 0]]*footStep 

    # Motor 34 and Motor 44
    initLeanAngleR = 0 # 正: 腳尖向下
    initLeanAngleL = 0 # 正: 腳尖向下

    leanAngleR1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]] # 2正往下壓 [0, 0, 0, 0+2, 0+2]
    leanAngleR2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]] #成功-2, -1, 0, 1, 1
    leanAngleR3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    leanAngleR = leanAngleR1+leanAngleR2*int(footStep/2-2)+leanAngleR3
    leanAngleL1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]] # # 成功[0+1, 0+1, 0+2, 0+2, 0+2]
    leanAngleL2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    leanAngleL3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    leanAngleL = leanAngleL1+leanAngleL2*int(footStep/2-2)+leanAngleL3

    # Motor 35 and Motor 45
    pedalRollAngleR1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]#8  # 成功 [0, 0, 0+2, 2+1, 4+1]
    pedalRollAngleR2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    pedalRollAngleR3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    pedalRollAngleR = pedalRollAngleR1+pedalRollAngleR2*int(footStep/2-2)+pedalRollAngleR3
    pedalRollAngleL1 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]] #正往右倒 # 成功[0, 0-2, 0-2, 0-2, 0-2]
    pedalRollAngleL2 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    pedalRollAngleL3 = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    pedalRollAngleL = pedalRollAngleL1+pedalRollAngleL2*int(footStep/2-2)+pedalRollAngleL3

    """ Data Preprocessing """
    B = [b1, b2]+[b3,b4]*int(footStep/2-2)+[b5,b6]
    Hip = [hip1, hip2]
    StepHeight = [[stepHeight1, 0]+[stepHeight3, 0]*int(footStep/2-2)+[stepHeight5, 0]]+\
                [[0, stepHeight2]+[0, stepHeight4]*int(footStep/2-2)+[0, stepHeight6]]
    StepSize = [stepSize1, stepSize2]+[stepSize3, stepSize4]*int(footStep/2-2)+[stepSize5, stepSize6]
    Shift = [shift1, shift2]
    InitLeanAngle = [initLeanAngleR, initLeanAngleL]
    LeanAngle = [leanAngleR, leanAngleL]
    YawAngle = [yawAngleR, yawAngleL]
    pedalRollAngle = [pedalRollAngleR, pedalRollAngleL]
    """ Parameters of robot """
    legLinkLength = [102, 357.95, 366.42, 29, 111.75]
    footHeight = 978
    zCoM = 674.5
    xCoM = 0
    d2 = 6 / 1000
    """ Generate Original Profile """
    kDSP = 0.5#雙腳著地時間比例
    period = 0.6#走一步的時間
    samplingTime = 0.01

    LIPM_motion = LIPM_motion_generator(rightFirst, forward, shift, turn)
    LIPM_motion.setRobot(legLinkLength, footHeight, zCoM, xCoM, d2)
    LIPM_motion.setParameters(B, Hip, StepHeight, StepSize, Shift, InitLeanAngle,
                            LeanAngle, YawAngle, pedalRollAngle)
    outputData = LIPM_motion.gaitGeneration(period=period,
                                            dt=samplingTime,
                                            footStep=footStep,
                                            kDSP=kDSP)
    """ Generate motion Data """
    initR = [0, 0, -0.35, 0.7, -0.35, 0]
    initL = [0, 0, -0.35, 0.7, -0.35, 0]
    initPose = [initR, initL]

    # 重要！單位！！
    if unit == 1:
        scaleR = [1, 180/np.pi, 1, 1, 1, 180/np.pi]
        scaleL = [1, 180/np.pi, 1, 1, 1, 180/np.pi]
    elif unit == 0:
        scaleR = [1, 1, 1, 1, 1, 1]
        scaleL = [1, 1, 1, 1, 1, 1]
    scale = [scaleR, scaleL]

    # 重要！馬達方向！！
    dirR = [1, -1, 1, 1, 1, -1]
    dirL = [1, -1, 1, 1, 1, -1]
    dir = [dirR, dirL]


    if motionGen == True: # motion 檔 
        currentFolder = os.path.dirname(os.path.abspath(__file__))
        motionFilePath = currentFolder + "/src/" + fileName
        LIPM_motion.writeFile(outputData, motionFilePath, initPose, scale, dir)
# joint_name = ['L_hip_yaw', 'L_hip_roll', 'L_hip_pitch', 'L_knee', 'L_ankle_pitch', 'L_ankle_roll', 'R_hip_yaw', 'R__hip_roll', 'R__hip_pitch', 'R__knee', 'R__ankle_pitch', 'R__ankle_roll']

    # 讀Motion檔
    motion = pd.read_csv('Cho/src/F2.csv',header=None,index_col=None)
    balance_step=50
    motion=Data_preprocess(motion,balance_step)
    if total_step!=generate_step:
        motion=motion.loc[0:balance_step+total_step*int(period/samplingTime)]

    one_step = int(period/samplingTime)
## motor one/two step delay
    # dynamix=[1,5,7,11]
    # for idx in dynamix:
    #     num = 20
    #     first_step = list(motion.iloc[ 0:150,idx])
    #     temp = first_step+[motion.iloc[150,idx]]*num+list(motion.iloc[150:,idx])
    #     temp = temp[0:len(temp)]
    #     motion[idx] = pd.DataFrame(temp,columns=[idx])


    # myact=[2,3,4]
    # for idx in myact:
    #     num = 20
    #     first_step = list(motion.iloc[ 0:150-num,idx])
    #     temp = first_step+list(motion.iloc[ 150:,idx])+[motion.iloc[len(motion.iloc[:,idx])-1,idx]]*num
    #     motion[idx] = pd.DataFrame(temp,columns=[idx])

##波峰波谷
    myact=[3,4,9,10]
    for idx in myact:
        if idx==3 or idx==9:
            num=16
            for region in range(0,8,2):
                # print(region)
                column = list(motion.iloc[50+region*one_step:50+(region+2)*one_step,idx])
                wave_idx = [column.index(max(column)),column.index(min(column))]
                # print(wave_idx)
                wave_idx.sort()
                motion.iloc[50+region*one_step+int(wave_idx[0]-num*3/4):50+region*one_step+int(wave_idx[0]+num/4),idx] = [column[wave_idx[0]]]*num
                motion.iloc[50+region*one_step+int(wave_idx[1]-num*3/4):50+region*one_step+int(wave_idx[1]+num/4),idx] = [column[wave_idx[1]]]*num
        elif idx==4 or idx==10:
            num=29
            for region in range(0,8,2):
                column = list(motion.iloc[50+region*one_step:50+(region+2)*one_step,idx])
                column1 = [motion.iloc[50,idx]]*len(column)
                wave_idx = [column.index(max(column))]
                wave_idx.sort()
                column1[int(wave_idx[0]-num/2+5):int(wave_idx[0]+num/2+5)] = [column[wave_idx[0]]]*num
                motion.iloc[50+region*one_step:50+(region+2)*one_step,idx] = column1
    pd.DataFrame(motion).to_csv('Cho/src/F2.csv',header=None,index=False)
#dynamix delay
    dynamix=[2,6,8,12]
    for idx in dynamix:
        num = 10
        temp = [0]*num+list(motion.iloc[ :,idx])
        temp = temp[0:len(temp)-num]
        motion[idx] = pd.DataFrame(temp,columns=[idx])
        motion.iloc[50+generate_step*one_step+num:,idx] = [0]*len(motion.iloc[50+generate_step*one_step+num:,idx])
# #myactuator delay
#     dynamix=[3,9]
#     for idx in dynamix:
#         num = 5
#         temp = list(motion.iloc[ :,idx])+[0]*num
#         temp = temp[num:len(temp)]
#         motion[idx] = pd.DataFrame(temp,columns=[idx])
    
    motion.iloc[:,2]-=deg2rad(init_l[0])
    motion.iloc[:,3]-=deg2rad(init_l[1])
    motion.iloc[:,4]-=deg2rad(init_l[2])
    motion.iloc[:,5]-=deg2rad(init_l[3])
    motion.iloc[:,6]-=deg2rad(init_l[4])

    motion.iloc[:,8]-=deg2rad(init_r[0])
    motion.iloc[:,9]-=deg2rad(init_r[1])
    motion.iloc[:,10]-=deg2rad(init_r[2])
    motion.iloc[:,11]-=deg2rad(init_r[3])
    motion.iloc[:,12]-=deg2rad(init_r[4])

    pd.DataFrame(motion).to_csv('Cho/src/F2_filter.csv',header=None,index=False)