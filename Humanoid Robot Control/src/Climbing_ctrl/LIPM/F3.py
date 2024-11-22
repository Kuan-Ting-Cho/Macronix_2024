from LIPM.src.LIPMMotionGenerator import LIPM_motion_generator
# from src.cmdGenerator import cmdGenerator
import numpy as np
import time


class Forward2:
    def __init__(self, file, alter_z):
        """ Common Setting """
        self.fileName = file    # outputName
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
        b1 = -0.215
        b2 = 0.211

        # Motor 31/35 and Motor 41/45
        # 正： 腰與腳踝彎曲方向一致 #調小重心轉換沒那麼過去!
        hip1 = 0.26
        hip2 = 0.24

        # Step Height
        stepHeight1 = 0.1
        stepHeight2 = 0.1

        # Forward Distance
        stepSize1 = 0.26
        stepSize2 = 0.

        # Lateral Displacement
        shift1 = [0.0, 0]
        shift2 = [0, 0.0]

        # Motor 30 and Motor 40
        yawAngleR = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開
        yawAngleL = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]  # 正: 往外開

        # Motor 34 and Motor 44
        initLeanAngleR = 0
        initLeanAngleL = 0
        leanAngleR = [[0, 0, 0, 0, 0], [0, -5, -4, -5, -3]]  # 正: 腳尖向up
        leanAngleL = [[0, 0, 0, 0, 0], [0, 3, 3, 0, 0]]  # 正: 腳尖向下

        # Motor 35 and Motor 45 
        pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0,  0, 0]]    # 正: 往內番
        pedalRollAngleL = [[0, 0, 0, 0, 0], [0, 0, -4, -4, 0]]   # 正: 往內番

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
        self.footHeight = 978
        self.zCoM = 674.5
        self.alter_zCoM = alter_z
        self.xCoM = 0
        self.d2 = 6 / 1000

        """ Generate Original Profile """
        self.kDSP = 0.4419 #雙腳著地時間比例
        self.period = 1 #走一步的時間
        self.samplingTime = 0.01

    def setParameter(self, x, alter_z):
        self.B = [x[0], x[1]]
        self.Hip = [x[2], x[3]]
        self.StepHeight = [[x[4], 0],
                        [0, x[5]]]
        if alter_z > 0: mode = 1
        elif alter_z == 0: mode = 2
        else : mode = 3
        
        if mode == 1:
            self.kDSP = 0.4419 #雙腳著地時間比例
            self.footHeight = 978
            self.StepSize = [x[6], 0]

            leanAngleR = [[0, 0, 0, 0, 0], [0, x[8], x[8]+1, x[9], x[9]+2]]  # 正: 腳尖向up
            leanAngleL = [[0, 0, 0, 0, 0], [0, x[10], x[11], 0, 0]]  # 正: 腳尖向下
            pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0,  0, 0]]    # 正: 往內番
            pedalRollAngleL = [[0, 0, 0, 0, 0], [0, 0, x[12], x[12], 0]]   # 正: 往內番

        elif mode == 2:
            self.kDSP = 0.68 #雙腳著地時間比例
            self.footHeight = 978
            self.StepSize = [x[6], -0.05]

            leanAngleR = [[0, 0, 0, 0, 0], [x[8], x[8]-1, x[9], x[9], 0]]  # 正: 腳尖向up
            leanAngleL = [[0, 0, 0, 0, 0], [0, x[10], x[11], x[11], 0]]  # 正: 腳尖向下
            pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0,  0, 0]]    # 正: 往內番
            pedalRollAngleL = [[0, 0, 0, 0, 0], [0, 0,  0,  0, 0]]   # 正: 往內番

        elif mode == 3:
            self.kDSP = 0.4419 #雙腳著地時間比例
            self.footHeight = 925
            self.StepSize = [x[6], x[7]]

            leanAngleR = [[0, 0, 0, 0, 0], [0, x[8], x[8], x[9], 0]]  # 正: 腳尖向up
            leanAngleL = [[0, 0, 0, 0, 0], [x[10], x[10], 0, x[11], x[11]]]  # 正: 腳尖向下
            pedalRollAngleR = [[0, 0, 0, 0, 0], [0, 0,  0,  0, 0]]    # 正: 往內番
            pedalRollAngleL = [[0, 0, 0, 0, 0], [0, 0,  0,  0, 0]]   # 正: 往內番
            
        self.LeanAngle = [leanAngleR, leanAngleL]
        self.pedalRollAngle = [pedalRollAngleR, pedalRollAngleL]
        self.alter_zCoM = alter_z

    def output_motion(self):

        LIPM_motion = LIPM_motion_generator(self.rightFirst, self.forward, self.shift, self.turn)

        LIPM_motion.setRobot(self.legLinkLength, self.footHeight, self.zCoM, self.xCoM, self.d2)
        LIPM_motion.setParameters(self.B, self.Hip, self.StepHeight, self.StepSize, self.Shift, self.InitLeanAngle,
                                self.LeanAngle, self.YawAngle, self.pedalRollAngle)
        tmp=time.time()

        outputData,Rd_end,Ld_end,CoM = LIPM_motion.gaitGeneration(period=self.period,
                                                dt=self.samplingTime,
                                                footStep=self.footStep,
                                                kDSP=self.kDSP,
                                                alter_zCoM=self.alter_zCoM)
        # print("F3",time.time()-tmp)

        """ Generate Motion Data """
        bias = 0.2 if self.alter_zCoM < 0 else 0
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
            motionFilePath = self.fileName
            LIPM_motion.writeFile(outputData, motionFilePath, initPose, scale, dir)
