from .LIPMFunction import StepSize2StrideLength, StrideLength2ZMPAmplitude, \
    ZMPAmplitudeSign, ModifiableXOSGRampDSPRampSSP, ModifiableYOSGRampDSPSinSSP, \
        flipsign, CalKLeanAngle, CompletedRGeneration, ModifiableFootGeneration, \
            OutputMotion

import numpy as np
import csv


class LIPM_motion_generator:
    def __init__(self, rightFirst, forward, shift, turn, periodAcc = 0, periodDec = 0):
        self.rightFirst = rightFirst
        self.forward = forward
        self.shift = shift
        self.turn = turn
        self.periodAcc = periodAcc
        self.periodDec = periodDec

    def setRobot(self, legLinkLength, footHeight, zCoM, xCoM, d2):
        self.legLinkLength = np.array(legLinkLength) * 0.001
        self.footHeight = footHeight * 0.001
        self.zCoM = zCoM * 0.001
        self.xCoM = xCoM * 0.001
        self.d2 = d2

    def setParameters(self, B, Hip, StepHeight, StepSize, Shift, InitLeanAngle,
                      LeanAngle, YawAngle, pedalRollAngle):
        self.B = np.array(B)
        self.hip1 = Hip[0]
        self.hip2 = Hip[1]
        self.stepHeight1 = np.array(StepHeight[0])
        self.stepHeight2 = np.array(StepHeight[1])
        self.stepSize = StepSize
        self.shift1 = Shift[0]
        self.shift2 = Shift[1]
        self.initLeanAngleR = np.array(InitLeanAngle[0])
        self.initLeanAngleL = np.array(InitLeanAngle[1])
        self.leanAngleR = np.array(LeanAngle[0])
        self.leanAngleL = np.array(LeanAngle[1])
        self.yawAngleR = np.array(YawAngle[0])
        self.yawAngleL = np.array(YawAngle[1])
        self.pedalRollAngleR = np.array(pedalRollAngle[0])
        self.pedalRollAngleL = np.array(pedalRollAngle[1])

    def gaitGeneration(self, period=2, dt=0.01, footStep=2, kDSP=0.4419, firstterrain=[0,0,0,0], finalterrain=[0,0,0,0]):
        self.period = period
        self.dt = dt
        self.footStep = footStep
        self.kDSP = kDSP
        self.firstterrain = firstterrain
        self.finalterrain = finalterrain
        g = 9.81
        deg2Rad = np.pi / 180
        k = 0.1
        tDSP = self.period * self.kDSP
        wn_T = np.sqrt(g / self.zCoM)  #

        firstLegLength, secondLegLength, length, a = StepSize2StrideLength(
            self.stepSize)
        ZMPAmplitude = StrideLength2ZMPAmplitude(length) #總長度
        ZMPAmplitude, a = ZMPAmplitudeSign(ZMPAmplitude, a, self.forward)
        CoMx, _, _ = ModifiableXOSGRampDSPRampSSP(ZMPAmplitude, a, k, 
                                                  self.period, self.dt, wn_T) #完整CoM X軌跡、速度、加速度
        CoMx[1:] = CoMx[1:] + self.xCoM
        b = self.rightFirst * flipsign(0.005, self.footStep) + (
            1 - self.rightFirst) * flipsign(-0.005, self.footStep)
        CoMy, _, _ = ModifiableYOSGRampDSPSinSSP(self.B, b, k, self.period, #完整CoM Y軌跡、速度、加速度
                                                 self.dt, wn_T, (~self.shift))

        k_LeanAngle = CalKLeanAngle(self.kDSP, self.footStep)
        leanAngleR = CompletedRGeneration(self.initLeanAngleR, k_LeanAngle,
                                          self.leanAngleR, self.period,
                                          self.dt)
        leanAngleL = CompletedRGeneration(self.initLeanAngleL, k_LeanAngle,
                                          self.leanAngleL, self.period,
                                          self.dt)
        leanAngleR = leanAngleR * deg2Rad
        leanAngleL = leanAngleL * deg2Rad

        yawAngleR = CompletedRGeneration(0, k_LeanAngle, self.yawAngleR,
                                            self.period, self.dt)
        yawAngleL = CompletedRGeneration(0, k_LeanAngle, self.yawAngleL,
                                            self.period, self.dt)
        yawAngleR = yawAngleR * deg2Rad
        yawAngleL = yawAngleL * deg2Rad

        pedalRollAngleR = CompletedRGeneration(0, k_LeanAngle, self.pedalRollAngleR,
                                            self.period, self.dt)
        pedalRollAngleL = CompletedRGeneration(0, k_LeanAngle, self.pedalRollAngleL,
                                            self.period, self.dt)
        pedalRollAngleR = pedalRollAngleR * deg2Rad
        pedalRollAngleL = pedalRollAngleL * deg2Rad

        if self.forward is False:
            firstLegLength = -firstLegLength
            secondLegLength = -secondLegLength

        if self.rightFirst:
            LengthHeightR = np.append([firstLegLength], [self.shift1], axis=0)
            LengthHeightR = np.append(LengthHeightR, [self.stepHeight1],
                                      axis=0)
            LengthHeightL = np.append([secondLegLength], [self.shift2], axis=0)
            LengthHeightL = np.append(LengthHeightL, [self.stepHeight2],
                                      axis=0)
        else:
            LengthHeightL = np.append([firstLegLength], [self.shift1], axis=0)
            LengthHeightL = np.append(LengthHeightL, [self.stepHeight1],
                                      axis=0)
            LengthHeightR = np.append([secondLegLength], [self.shift2], axis=0)
            LengthHeightR = np.append(LengthHeightR, [self.stepHeight2],
                                      axis=0)

        footR = ModifiableFootGeneration(LengthHeightR, self.period, self.dt, #p.20
                                         tDSP)
        footL = ModifiableFootGeneration(LengthHeightL, self.period, self.dt,
                                         tDSP)
        t = np.linspace(self.dt,
                        self.footStep * self.period,
                        int(self.footStep * self.period / self.dt),
                        endpoint=True)
        zcSinusoid = -0.0012 * np.cos(2 * np.pi * t / self.period)
        zr = [[]]
        zl = [[]]
        aa = 0.0001
        for i in range(140):
            if i > 30 and i < 50:
                zr[0].append(aa/20*(i-30))
            elif i > 100 and i < 120:
                zr[0].append(aa/20*(120-i))
            elif i>=50 and i<=100:
                zr[0].append(aa)
            else:
                zr[0].append(0.0)
        # for i in range(140):
        #     if i < 110:
        #         zl[0].append(0)
        #     elif i == 139:
        #         zl[0].append(0.0)
        #     else:
        #         zl[0].append(0.02)
        xPR = footR[0, :] - CoMx  
        yPR = footR[1, :] - CoMy 
        zPR = footR[2, :] - (zcSinusoid + self.footHeight) -zr
        xPL = footL[0, :] - CoMx
        yPL = footL[1, :] - CoMy
        zPL = footL[2, :] - (zcSinusoid + self.footHeight) 
        PR = np.append(xPR, yPR, axis=0)
        PR = np.append(PR, zPR, axis=0)
        PL = np.append(xPL, yPL, axis=0)
        PL = np.append(PL, zPL, axis=0)
        outputData = OutputMotion(PR, PL, self.hip1, self.hip2, leanAngleR,
                                  leanAngleL, yawAngleR, yawAngleL, pedalRollAngleR, pedalRollAngleL,
                                  self.legLinkLength, self.turn,
                                  self.rightFirst, self.period, self.dt, self.firstterrain, self.finalterrain)
        return outputData

    def writeFile(self, data, fileName, initPose, scale, dir):
        initPose = np.append(initPose[0], initPose[1], axis =0)
        scale = np.append(scale[0], scale[1], axis =0)
        dir = np.append(dir[0], dir[1], axis =0)

        fileNum = int(self.footStep/2)
        rowNum_per2step = int(data.shape[0]/fileNum)
        for i in range(fileNum+1):
            outputfilepath = ""
            if i == 0 or self.footStep==2:
                writeData = data
                outputfilepath = fileName + ".csv"
            else:
                writeData = data[(i-1)*rowNum_per2step:i*rowNum_per2step]
                outputfilepath = fileName + "_" + str(i) + ".csv"
            
            with open(outputfilepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                for row in writeData:
                    row = initPose + scale * dir * row
                    writer.writerow(row)

