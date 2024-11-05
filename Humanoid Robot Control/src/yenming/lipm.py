from src.LIPMMotionGenerator import LIPM_motion_generator
# from src.cmdGenerator import cmdGenerator
import numpy as np
from dataprocess import Data_preprocess
import pandas as pd
import csv

class Forward2:
    def __init__(self):
        """ Common Setting """
        self.fileName = 'F2_200'    # outputName
        self.footStep = 2
        self.motionGen = True
        self.cmdGen = False
        self.unit = 0 # 0 => for simulation; 1 => for real world
        
        """ Define type of motion """
        self.rightFirst = True
        self.forward = True
        self.shift = False
        self.turn = True

        """ Setting of LIPM parameters """
        # Amplitude of swing
        # b1 = -0.215
        # b2 = 0.211
        b1 = -0.23
        b2 = 0.21+0.03 -0.02
        # Motor 31/35 and Motor 41/45
        # 正： 腰與腳踝彎曲方向一致 #調小重心轉換沒那麼過去!
        hip1 = 0.2
        hip2 = -0.2

        # Step Height
        stepHeight1 = 0.055+0.02
        stepHeight2 = 0.055+0.008

        # Forward Distance
        stepSize1 = 0.07
        stepSize2 = 0.0

        # Lateral Displacement
        shift1 = [0.0, 0]
        shift2 = [0, 0.0]

        # Motor 30 and Motor 40
        yawAngleR = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開
        yawAngleL = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開

        # Motor 34 and Motor 44
        initLeanAngleR = 0
        initLeanAngleL = 0
        leanAngleR = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 負: 腳尖向up
        leanAngleL = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 腳尖向下

        # Motor 35 and Motor 45 
        pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0,  0, 0]]    # 正: 往內番
        pedalRollAngleL = [[0, 0, 0, 0, 0], [0, 0, 0,0, 0]]   # 正: 往內番

        """ Data Preprocessing """
        self.B = [b1, b2]
        self.Hip = [hip1, hip2]
        self.StepHeight = [[stepHeight1, 0],
                    [0, stepHeight2]]
        self.StepSize = [stepSize1, stepSize2]
        self.Shift = [shift1, shift2]
        self.InitLeanAngle = [initLeanAngleR, initLeanAngleL]
        self.LeanAngle = [leanAngleR, leanAngleL]
        self.YawAngle = [yawAngleR, yawAngleL]
        self.pedalRollAngle = [pedalRollAngleR, pedalRollAngleL]

        """ Parameters of robot """
        self.legLinkLength = [102, 357.95, 366.42, 29, 111.75]
        self.footHeight = 978  # ori 978
        self.zCoM = 674.5
        self.xCoM = 0
        self.d2 = 6 / 1000

        """ Generate Original Profile """
        self.kDSP = 0.5 #雙腳著地時間比例
        self.period = 0.7 #走一步的時間
        self.samplingTime = 0.01
        self.firstterrain = [1, 0, 0, 0] #左腳開始pitch、左腳開始roll、右腳開始pitch、右腳開始roll
        self.finalterrain = [1, 0, 0, 0] #左腳結束pitch、左腳結束roll、右腳結束pitch、右腳結束roll

    def output_motion(self,desired,footstep,isflat):
        LIPM_motion = LIPM_motion_generator(self.rightFirst, self.forward, self.shift, self.turn)
        LIPM_motion.setRobot(self.legLinkLength, self.footHeight, self.zCoM, self.xCoM, self.d2)
        LIPM_motion.setParameters(self.B, self.Hip, self.StepHeight, self.StepSize, self.Shift, self.InitLeanAngle,
                                self.LeanAngle, self.YawAngle, self.pedalRollAngle)
        outputData = LIPM_motion.gaitGeneration(period=self.period,
                                                dt=self.samplingTime,
                                                footStep=self.footStep,
                                                kDSP=self.kDSP,
                                                firstterrain=self.firstterrain,
                                                finalterrain=self.finalterrain)
    
        """ Generate Motion Data """
        bias = 0
        initR = list(desired[6:]) #臀部調小/膝蓋調大/腳踝調小:後傾
        initL = list(desired[:6])  #
        initPose = [initR, initL]

        # 重要！單位！！ 角度弧度轉換
        if self.unit == 1:
            scaleR = [1, 180/np.pi, 1, 1, 1, 180/np.pi]
            scaleL = [1, 180/np.pi, 1, 1, 1, 180/np.pi]
        elif self.unit == 0:
            scaleR = [1, 1, 1, 1, 1, 1]
            scaleL = [1, 1, 1, 1, 1, 1]
        scale = [scaleR, scaleL]

        # 重要！馬達方向！！ 模擬跟實體可能有方向差
        dirR = [1, -1, 1, 1, 1, -1]
        dirL = [1, -1, 1, 1, 1, -1]
        dir = [dirR, dirL]

        if self.motionGen == True: # Motion 檔 
            motionFilePath = 'yenming/src/' + self.fileName
            LIPM_motion.writeFile(outputData, motionFilePath, initPose, scale, dir)
            with open('yenming/src/finalterrain.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(self.finalterrain)

        # temp return
        motion = pd.read_csv('yenming/src/F2_200.csv',header=None,index_col=None)
        motion=Data_preprocess(motion,footstep,isflat)
        # pd.DataFrame(motion).to_csv('yenming/src/F2_200.csv',header=None,index=False)