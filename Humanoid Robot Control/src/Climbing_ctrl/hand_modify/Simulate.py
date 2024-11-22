import mujoco_py
import numpy as np
import math
import pandas as pd


column = [
    "trunk",
    "L_hip_yaw",
    "L_hip_roll",
    "L_hip_pitch",
    "L_knee",
    "L_ankle_pitch",
    "L_ankle_roll",
    "R_hip_yaw",
    "R_hip_roll",
    "R_hip_pitch",
    "R_knee",
    "R_ankle_pitch",
    "R_ankle_roll",
]
pd.set_option("display.max_rows", 10000)
pi = math.pi

# Pressure sensor position relative to base
p_sensor_pos = [ #DSP
    [0.0755, -0.15225],  # RFR
    [0.0755, -0.07225],  # RFL
    [-0.0545, -0.15225],  # RBR
    [-0.0545, -0.07225],  # RBL
    [0.0755, 0.07225],  # LFR
    [0.0755, 0.15225],  # LFL
    [-0.0545, 0.07225],  # LBR
    [-0.0545, 0.15225],  # LBL
]

RFR = [0.0755, -0.15225, 0.0044]
RFL = [0.0755, -0.07225, 0.0044]
RBR = [-0.0545, -0.15225, 0.0044]
RBL = [-0.0545, -0.07225, 0.0044]
LFR = [0.0755, 0.07225, 0.0044]
LFL = [0.0755, 0.15225, 0.0044]
LBR = [-0.0545, 0.07225, 0.0044]
LBL = [-0.0545, 0.15225, 0.0044]


# 機器人估重 19.74 Kg
def deg2rad(angle):
    global pi
    return angle * pi / 180


def rad2deg(radius):
    global pi
    return radius / pi * 180


# 將弧度換算角度進行控制運算
def PID_control(kp, kv, ki, qpos, qvel, controller, acc_err):
    signal = []
    for i in range(len(controller)):
        if i == 2 or i==8:
            kp = 1000
        elif i == 6 or i==12:
            kp = 100
        else:
            kp = 160

        acc_err[i] = acc_err[i] + qpos[i] - controller[i]
        sig = -kp * (qpos[i] - controller[i]) - ki * acc_err[i] - kv * qvel[i]
        if np.isnan(sig):
            sig = 0
        elif sig > 35:
            sig = 35
        elif sig < -35:
            sig = -35
        signal.append(sig)
    return signal, acc_err

def quart2rpy(quart):
    w, x, y, z = quart
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw

# 將Motion_data做線性插值，回傳插值動作
def Linear_interp(motion, index1, index2, num):
    motion1 = motion.loc[index1].tolist()
    motion2 = motion.loc[index2].tolist()
    motion_interp = np.linspace(motion1, motion2, num)

    return motion_interp


# 將Motion_data做線性插值，回傳加入插值動作的新Motion_data
def Data_preprocess(motion):
    # Swap left and right leg cmd
    columns_titles = [6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5]
    motion = motion.reindex(columns=columns_titles)

    # 加一全為零的新欄(for trunk)
    df = pd.DataFrame([deg2rad(0)] * len(motion.iloc[:, 0]))
    motion = pd.concat([df, motion], axis=1, ignore_index=True)
    # 加一全為零的新列(站直)
    df1 = pd.DataFrame([0.0] * len(motion.loc[0])).T
    motion = pd.concat([df1, motion], axis=0, ignore_index=True)

    # 在站直與初始蹲姿間做插值
    df2 = pd.DataFrame(Linear_interp(motion, 0, 1, 100))
    motion = pd.concat([df2, motion], axis=0, ignore_index=True)
    motion = motion.drop(100).reset_index(drop=True)

    # 加入站直動作平衡所需的Step(這邊給30 time steps)
    df3 = pd.DataFrame(0, index=range(30), columns=range(13))
    motion = pd.concat([df3, motion], axis=0, ignore_index=True)

    # 在final與初始蹲姿間做插值
    df4 = pd.DataFrame(Linear_interp(motion,328,329,100))
    motion=pd.concat([motion, df4],axis=0,ignore_index=True)
    motion = motion.drop(329).reset_index(drop=True)        
    motion = pd.concat([motion, motion.iloc[[-1]]], axis=0, ignore_index=True)     

    return motion


class Simulation:
    def __init__(self):
        # 載入URDF文件
        self.model = mujoco_py.load_model_from_path("linkage_robot.xml")
        # 讀Motion檔
        motion_csv = pd.read_csv(
            "./motordata/forward_data_train.csv", header=None, index_col=None
        )

        self.motion = Data_preprocess(motion_csv)

        # 創建Mujoco仿真環境
        self.sim = mujoco_py.MjSim(self.model)
        # 顯示Mujoco仿真視窗
        # self.viewer = mujoco_py.MjViewer(self.sim)

    def cal_cop(self, right, left, base_pos):
        """This function calculte center of presure

        Args:
            right (list): right sole presure
            left (list): left sole presure
            pos (list): presure sensor position
        """
        # print("LEFT:", left)
        # print("RIGHT: ",right)
        if not any(right) and not any(left):  # fall down
            phase = 3
            return 0, 0, phase
        elif not any(right):  # single support on left
            phase = 2
        elif not any(left):  # single support on right
            phase = 1
        else:  # double support
            phase = 0


        x_cop = 0
        y_cop = 0
        for i in range(len(right)):
            x_cop += right[i] * (base_pos[0]+p_sensor_pos[i][0]) + left[i] * (base_pos[0]+p_sensor_pos[i + 4][0]) #absolute base
            y_cop += right[i] * (base_pos[1]+p_sensor_pos[i][1]) + left[i] * (base_pos[1]+p_sensor_pos[i + 4][1])

        x_cop /= (sum(right) + sum(left))
        y_cop /= (sum(right) + sum(left))

        return x_cop, y_cop, phase

    def track_cop(self, base, x_cop, y_cop):
        """Calculate cop error

        Args:
            base (list): Base position, could be Base or Ankle roll
            x_cop (double): Current x COP
            y_cop (double): Current y COP

        Returns:
            double: COP error
        """
        return (x_cop-base[0])**2 + (y_cop-base[1])**2
    
    def is_healthy(self):
        """Check the robot is healthy or not

        Returns:
            bool: fall or not
        """
        is_healthy = True
        base_roll, base_pitch, base_yaw = quart2rpy(self.sim.data.qpos[3:7])
        if base_pitch < -0.785 or base_pitch > 0.785 or base_roll < -0.785 or base_roll > 0.785 \
            or not (-0.32 < base_yaw < 0.32):
            is_healthy = False

        return is_healthy

    def go_sim1(self):
        # 以下給角度 (單位：degree) L3 = L8, L9 = L_ankle_pitch, R3 = R8, R9 = -R_ankle_pitch
        init_pos = [
            0,
            0,
            1.0301,  # 機器人base起始位置 (不用動)
            1,
            0,
            0,
            0,  # 機器人起始四元數 (不用動)
            0,  # trunck        ,7  ,7
            0,  # L_hip_yaw     ,8  ,8
            0,  # L_hip_roll    ,9  ,9
            0,  # L_hip_pitch   ,10 ,10
            0,  # L3            ,11 ,11
            0,  # L4            ,12 ,12
            # 0,          # L6            ,13 ,13
            # 0,          # L7            ,14 ,14
            0,  # L8            ,15 ,15
            0,  # L_ankle_pitch ,16 ,16
            0,  # L_ankle_roll  ,17 ,17
            # 0,          # L_f_front     ,18
            # 0,          # L_f_back      ,19
            0,  # L9            ,20 ,18
            0,  # L10           ,21 ,19
            0,  # R_hip_yaw     ,22 ,20
            0,  # R_hip_roll    ,23 ,21
            0,  # R_hip_pitch   ,24 ,22
            0,  # R3            ,25 ,23
            0,  # R4            ,26 ,24
            # 0,          # R6            ,27 ,25
            # 0,          # R7            ,28 ,26
            0,  # R8            ,29 ,27
            0,  # R_ankle_pitch ,30 ,28
            0,  # R_ankle_roll  ,31 ,29
            # 0,          # R_f_front     ,32
            # 0,          # R_f_back      ,33
            0,  # R9            ,34 ,30
            0,  # R10           ,35 ,31
        ]

        # 以下給想要控制到的角度 (單位：degree)
        controller = [
            0,  # trunck
            0,  # L_hip_yaw
            0,  # L_hip_roll
            0,  # L_hip_pitch
            0,  # L8
            0,  # L_ankle_pitch
            0,  # L_ankle_roll
            0,  # R_hip_yaw
            0,  # R_hip_roll
            0,  # R_hip_pitch
            0,  # R8
            0,  # R_ankle_pitch
            0,  # R_ankle_roll
        ]
        acc_err = [0] * len(controller)

        # 給機器人 initial position
        init_pos[7:] = list(deg2rad(np.array(init_pos[7:])))
        self.sim.data.qpos[:] = init_pos

        # 給機器人 控制目標
        kp = 160
        kv = 0.6
        ki = 0.1

        for i in range(130):
            pos = [
                self.sim.data.qpos[7],
                self.sim.data.qpos[8],
                self.sim.data.qpos[9],
                self.sim.data.qpos[10],
                self.sim.data.qpos[13],
                self.sim.data.qpos[14],
                self.sim.data.qpos[15],
                self.sim.data.qpos[18],
                self.sim.data.qpos[19],
                self.sim.data.qpos[20],
                self.sim.data.qpos[23],
                self.sim.data.qpos[24],
                self.sim.data.qpos[25],
            ]
            vel = [
                self.sim.data.qvel[6],
                self.sim.data.qvel[7],
                self.sim.data.qvel[8],
                self.sim.data.qvel[9],
                self.sim.data.qvel[12],
                self.sim.data.qvel[13],
                self.sim.data.qvel[14],
                self.sim.data.qvel[17],
                self.sim.data.qvel[18],
                self.sim.data.qvel[19],
                self.sim.data.qvel[22],
                self.sim.data.qvel[23],
                self.sim.data.qvel[24],
            ]

            ctrl = self.motion.loc[i].tolist()
            self.sim.data.ctrl[:], acc_err = PID_control(kp, kv, ki, pos, vel, ctrl, acc_err)

            self.sim.step()

    def go_sim2(self, i):
        # 給機器人 控制目標
        kp = 160
        kv = 0.6
        ki = 0.1

        cop_err = 0
        acc_err = [0] * 13
        base_acc = [0] * 3


        pos = [
            self.sim.data.qpos[7],
            self.sim.data.qpos[8],
            self.sim.data.qpos[9],
            self.sim.data.qpos[10],
            self.sim.data.qpos[13],
            self.sim.data.qpos[14],
            self.sim.data.qpos[15],
            self.sim.data.qpos[18],
            self.sim.data.qpos[19],
            self.sim.data.qpos[20],
            self.sim.data.qpos[23],
            self.sim.data.qpos[24],
            self.sim.data.qpos[25],
        ]
        vel = [
            self.sim.data.qvel[6],
            self.sim.data.qvel[7],
            self.sim.data.qvel[8],
            self.sim.data.qvel[9],
            self.sim.data.qvel[12],
            self.sim.data.qvel[13],
            self.sim.data.qvel[14],
            self.sim.data.qvel[17],
            self.sim.data.qvel[18],
            self.sim.data.qvel[19],
            self.sim.data.qvel[22],
            self.sim.data.qvel[23],
            self.sim.data.qvel[24],
        ]

        ctrl = self.motion.loc[i].tolist()
        self.sim.data.ctrl[:], acc_err = PID_control(kp, kv, ki, pos, vel, ctrl, acc_err)

        self.sim.step()

        base_roll, base_pitch, base_yaw = quart2rpy(self.sim.data.qpos[3:7])
        z_pos = self.sim.data.qpos[2]
        R_z_pos = self.sim.data.get_body_xpos("R_ankle_roll")[2]
        L_z_pos = self.sim.data.get_body_xpos("L_ankle_roll")[2]

        x_pos = self.sim.data.qpos[0]
        y_pos = self.sim.data.qpos[1]
        base_pos = [x_pos, y_pos]
        x_cop, y_cop, phase = self.cal_cop(self.sim.data.sensordata[:4], self.sim.data.sensordata[4:], base_pos)

        base_acc[0] = self.sim.data.qacc[0]
        base_acc[1] = self.sim.data.qacc[1]
        base_acc[2] = self.sim.data.qacc[2] if i > 132 else 0
        
        if phase == 0:
            cop_err += self.track_cop(base_pos, x_cop, y_cop)
        
        elif phase == 1:
            right_pos = self.sim.data.get_body_xpos("R_ankle_roll")[:2]
            cop_err += self.track_cop(right_pos, x_cop, y_cop)

        elif phase == 2:
            left_pos = self.sim.data.get_body_xpos("L_ankle_roll")[:2]
            cop_err += self.track_cop(left_pos, x_cop, y_cop)
        
        else:
            return 3, cop_err, x_pos, y_pos, z_pos, R_z_pos, L_z_pos
        
        # self.viewer.render()
        # print(self.sim.data.get_body_xpos("base")[2])S
        is_healthy = self.is_healthy()


        return is_healthy, cop_err, x_pos, y_pos, z_pos, R_z_pos, L_z_pos

