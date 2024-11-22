from src.LIPMMotionGenerator import LIPM_motion_generator
import os
import numpy as np
from src.cmdGenerator import cmdGenerator
import math
""" Common Setting """
fileName = 'F6_200'    # outputName
footStep = 6
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
b1 = -90 * 0.0027
b2 = 90 * 0.0023
b3 = -90 * 0.01
b4 = 90 * 0.0023
b5 = -90 * 0.0027
b6 = 90 * 0.0023

# Motor 31/35 and Motor 41/45
# 正： 腰與腳踝彎曲方向一致
hip1 = 0.4
hip2 = 0.4

# Step Height
stepHeight1 = 0.1
stepHeight2 = 0.1
stepHeight3 = 0.1
stepHeight4 = 0.12
stepHeight5 = 0.12
stepHeight6 = 0.1

# Forward Distance
stepSize1 = 0.15
stepSize2 = 0.25
stepSize3 = 0.25
stepSize4 = 0.25
stepSize5 = 0.25
stepSize6 = 0.15

# Lateral Displacement
shift1 = np.array([0, 0, 0, 0, 0, 0])
shift2 = np.array([0, 0, 0, 0, 0, 0])

# Motor 30 and Motor 40
yawAngleR = [[0, 0, 0, 0, 0],   # 正: 往外開
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0]]
yawAngleL = [[0, 0, 0, 0, 0],   # 正: 往內開
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0]]

# Motor 34 and Motor 44
initLeanAngleR = 0
initLeanAngleL = 0
leanAngleR = [[0, 0, 0, 0, 0], 
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0]]
leanAngleL = [[0, 0, 0, 0, 0], 
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0]]

# Motor 35 and Motor 45
pedalRollAngleR = [[0, 0, 0, 0, 0],  # 正: 往外番
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0]]
pedalRollAngleL = [[0, 0, 0, 0, 0], 
                   [0, -10, -10, -10, 0],
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0]]
""" Data Preprocessing """
B = [b1, b2, b3, b4, b5, b6]
Hip = [hip1, hip2]
StepHeight = [[stepHeight1, 0, stepHeight3, 0, stepHeight5, 0],
              [0, stepHeight2, 0, stepHeight4, 0, stepHeight6]]
StepSize = [stepSize1, stepSize2, stepSize3, stepSize4, stepSize5, stepSize6]
Shift = [shift1, shift2]
InitLeanAngle = [initLeanAngleR, initLeanAngleL]
LeanAngle = [leanAngleR, leanAngleL]
YawAngle = [yawAngleR, yawAngleL]
pedalRollAngle = [pedalRollAngleR, pedalRollAngleL]
""" Parameters of robot """
legLinkLength = [102, 357.95, 366.42, 29, 111.75]
footHeight = 1042.7
zCoM = 674.5
xCoM = 0
d2 = 6 / 1000
""" Generate Original Profile """
kDSP = 0.68 #雙腳著地時間比例
period = 1 #走一步的時間
samplingTime = 0.01

LIPM_motion = LIPM_motion_generator(rightFirst, forward, shift, turn)
LIPM_motion.setRobot(legLinkLength, footHeight, zCoM, xCoM, d2)
LIPM_motion.setParameters(B, Hip, StepHeight, StepSize, Shift, InitLeanAngle,
                          LeanAngle, YawAngle, pedalRollAngle)
outputData = LIPM_motion.gaitGeneration(period=period,
                                        dt=samplingTime,
                                        footStep=footStep,
                                        kDSP=kDSP)
""" Generate Motion Data """
initR = [0, 0, -0.35, 0.7, -0.35, 0]
initL = [0, 0, -0.35, 0.7, -0.35, 0]
initPose = [initR, initL]

# 重要！單位！！
if unit == 1:
    scaleR = [1, 180/math.pi, 1, 1, 1, 180/math.pi]
    scaleL = [1, 180/math.pi, 1, 1, 1, 180/math.pi]
elif unit == 0:
    scaleR = [1, 1, 1, 1, 1, 1]
    scaleL = [1, 1, 1, 1, 1, 1]
scale = [scaleR, scaleL]

# 重要！馬達方向！！
dirR = [1, -1, 1, 1, 1, -1]
dirL = [1, -1, 1, 1, 1, -1]
dir = [dirR, dirL]

