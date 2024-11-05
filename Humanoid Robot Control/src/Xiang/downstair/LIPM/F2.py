from LIPM.src.LIPMMotionGenerator import LIPM_motion_generator
import numpy as np
import pandas as pd

class Forward2:
    def __init__(self, alter_z):
        """ Common Setting """
        self.fileName = 'F2_200'    # outputName
        self.footStep = 2
        self.motionGen = True
        self.cmdGen = False
        self.unit = 0 # 0 => for simulation; 1 => for real world
        self.alter_z = alter_z
        print(self.alter_z)
        
        """ Define type of motion """
        self.rightFirst = True
        self.forward = True
        self.shift = False
        self.turn = True


        if self.alter_z == -0.04:
            """ Setting of LIPM parameters """
            # Amplitude of swing
            b1 = -0.215 - 0.07 
            b2 = 0.231+0.015+0.035+0.02

            # Motor 31/35 and Motor 41/45
            # 正： 腰與腳踝彎曲方向一致 #調小重心轉換沒那麼過去!
            hip1 = 0.15
            hip2 = 0.24

            # Step Height
            stepHeight1 = 0.06+0.015
            stepHeight2 = 0.1+0.015

            # Forward Distance
            stepSize1 = 0.28-0.04
            stepSize2 = -0.0

            # Lateral Displacement
            shift1 = [0.0, 0]
            shift2 = [0, 0.0]

            # Motor 30 and Motor 40
            yawAngleR = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開
            yawAngleL = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開

            # Motor 34 and Motor 44
            initLeanAngleR = 0
            initLeanAngleL = 0
            leanAngleR = [[0, 0, 0, 0, 0], [-4, -3, -3, -2, 0]]  # 正: 腳尖向down
            leanAngleL = [[0, +0, +0, 0, 0], [-2,  0, 0, 0, 0]]  # 正: 腳尖向下


            # Motor 35 and Motor 45 
            pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0, 0, 0]]    # 正: 往內番
            pedalRollAngleL = [[0, -0, -0, -0, 0], [0, 0,  0,  0, 0]]   # 正: 往內番

        elif self.alter_z == -0.053:
            """ Setting of LIPM parameters """
            # Amplitude of swing
            b1 = -0.215 - 0.07 
            b2 = 0.231+0.015+0.035+0.02

            # Motor 31/35 and Motor 41/45
            # 正： 腰與腳踝彎曲方向一致 #調小重心轉換沒那麼過去!
            hip1 = 0.15
            hip2 = 0.24

            # Step Height
            stepHeight1 = 0.06+0.015
            stepHeight2 = 0.1+0.015

            # Forward Distance
            stepSize1 = 0.28-0.04
            stepSize2 = -0.0

            # Lateral Displacement
            shift1 = [0.0, 0]
            shift2 = [0, 0.0]

            # Motor 30 and Motor 40
            yawAngleR = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開
            yawAngleL = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開

            # Motor 34 and Motor 44
            initLeanAngleR = 0
            initLeanAngleL = 0
            leanAngleR = [[0, 0, 0, 0, 0], [-4, -3, -3, -0, 0]]  # 正: 腳尖向down
            leanAngleL = [[0, 2, 1, 1, 0], [-3,  0, 0, 0, 0]]  # 正: 腳尖向下


            # Motor 35 and Motor 45 
            pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0, 0, 0]]    # 正: 往內番
            pedalRollAngleL = [[0, -0, -0, -0, 0], [0, 0,  0,  0, 0]]   # 正: 往內番


        elif self.alter_z == -0.09 or self.alter_z == -0.056:
            """ Setting of LIPM parameters """
            # Amplitude of swing
            b1 = -0.215 - 0.07 
            b2 = 0.231+0.015+0.035+0.02

            # Motor 31/35 and Motor 41/45
            # 正： 腰與腳踝彎曲方向一致 #調小重心轉換沒那麼過去!
            hip1 = 0.15
            hip2 = 0.24

            # Step Height
            stepHeight1 = 0.06+0.015
            stepHeight2 = 0.1+0.015

            # Forward Distance
            stepSize1 = 0.28-0.04
            stepSize2 = -0.0

            # Lateral Displacement
            shift1 = [0.0, 0]
            shift2 = [0, 0.0]

            # Motor 30 and Motor 40
            yawAngleR = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開
            yawAngleL = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開

            # Motor 34 and Motor 44
            initLeanAngleR = 0
            initLeanAngleL = 0
            leanAngleR = [[0, 0, 0, 0, 0], [-4, -4, -3, -0, 0]]  # 正: 腳尖向down
            leanAngleL = [[0, 0, +0.5, 0, 0], [0,  0, 0, 0, 0]]  # 正: 腳尖向下


            # Motor 35 and Motor 45 
            pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0, 0, 0]]    # 正: 往內番
            pedalRollAngleL = [[0, -0, -1, -1, 0], [0, 0,  0,  0, 0]]   # 正: 往內番

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
        self.zCoM = 674.5
        self.xCoM = 0
        self.d2 = 6 / 1000

        if self.alter_z>0:
            self.footHeight = 978  # ori 978
            self.kDSP = 0.4419 #雙腳著地時間比例
        elif self.alter_z==0:
            self.footHeight = 978  # ori 978
            self.kDSP = 0.68 #雙腳著地時間比例
        else:
            self.footHeight = 925  # ori 978
            self.kDSP = 0.4419 #雙腳著地時間比例

        """ Generate Original Profile """
        self.period = 0.7 #走一步的時間
        self.samplingTime = 0.01

    def output_motion(self):
        LIPM_motion = LIPM_motion_generator(self.rightFirst, self.forward, self.shift, self.turn)
        LIPM_motion.setRobot(self.legLinkLength, self.footHeight, self.zCoM, self.xCoM, self.d2)
        LIPM_motion.setParameters(self.B, self.Hip, self.StepHeight, self.StepSize, self.Shift, self.InitLeanAngle,
                                self.LeanAngle, self.YawAngle, self.pedalRollAngle)
        outputData,Rd_end,Ld_end,CoM = LIPM_motion.gaitGeneration(period=self.period,
                                                dt=self.samplingTime,
                                                footStep=self.footStep,
                                                kDSP=self.kDSP,
                                                alter_z=self.alter_z)
    

        """ Generate Motion Data """
        bias = 0.1
        initR = [0, 0, -0.35-bias, 0.7+2*bias, -0.35-bias, 0] #臀部調小/膝蓋調大/腳踝調小:後傾
        initL = [0, 0, -0.35-bias, 0.7+2*bias, -0.35-bias, 0] #
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
            motionFilePath = './Xiang/downstair/motordata/' + self.fileName
            LIPM_motion.writeFile(outputData, motionFilePath, initPose, scale, dir)
    
