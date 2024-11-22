import gym
from gym import spaces
from LIPM.F3 import Forward2
from Real import Robot
import math
import numpy as np
from Xiang.function import *
import time
import threading


global PI
PI = math.pi


class RolyEnv(gym.Env):
    def __init__(self, render=False, file=None):
        super(RolyEnv, self).__init__()
        self.render = render
        self.filename = file
        self.z_CoM = 0
        self.action_mode = 2
        #self.alter_CoM = np.array([0.07, 0.09, 0.1, 0, -0.036, -0.048, 0, -0.06, 0], dtype=np.float32)
        self.alter_CoM = np.array([0.07, 0.09, 0.1, 0, -0.06, -0.048, 0, -0.036, 0], dtype=np.float32) 

        
        '''Declare a np_random'''
        self.np_random = None
        self._reset_noise_scale = 1e-2

        self.done = False
        self.fall = False

        '''LIPM frame slice'''
        self.N = 20
        self.num = 0 # _th group
        self.flagEndOneStep = True
        self.step_count = 0  # record every episode how many step robot takes
        self.complete_one = False
        self.stair_porgress = 0
        self.action_slice = list(range(130,480))

        # self.x = np.array([-0.215, 0.211, 0.26, 0.24, 0.13, 0.13, 0.26, -5, -5, 5, 5, -4], dtype=np.float32)
        self.Robot = Robot(self.filename)
        self.Robot.set_motion()
        self.Robot.go_sim1()  # go to squat state

        self.quat = np.array([0]*4, dtype=np.float32)
        self.acc = np.array([0]*3, dtype=np.float32)
        self.pos = np.array([0]*13, dtype=np.float32)
        self.vel = np.array([0]*13, dtype=np.float32)

        self.basequat = np.array([0]*4*5, dtype=np.float32)
        self.baseacc = np.array([0]*3*5, dtype=np.float32)
        self.obs_height = np.array([0]*20, dtype=np.float32)
        # observation_space 表示在一個step後可觀察到的資訊( Ex.觀察機器人質心位置(x, y, z) = 3 dims)。
        base_low = np.array([-1] * 4 * 5, dtype=np.float32)  # base quat
        base_high = np.array([1] * 4 * 5, dtype=np.float32)

        acc_low = np.array([-10] * 3 * 5, dtype=np.float32)  # base acc
        acc_high = np.array([10] * 3 * 5, dtype=np.float32)

        motor_low = np.array([-100] * 26, dtype=np.float32) # pos 26
        motor_high = np.array([100] * 26, dtype=np.float32)

        stair_heightL = np.array([-0.1] * 20, dtype=np.float32)
        stair_heightH = np.array([0.10] * 20, dtype=np.float32)


        self.observation_space = spaces.Box(
            #                   Base pos,  Foot z position,      motor_status
            #                   x,y,z,w,     x_d, y_d, z_d,
            low=np.concatenate((base_low, acc_low, motor_low, stair_heightL), axis=0),
            high=np.concatenate((base_high, acc_high, motor_high, stair_heightH), axis=0),
        )
                        #0 - B1,  1-B2,  2-h1,  3-h2,  4-H1,  5-H2,  6-S1,   7-S2, LR1, LR2,  LL1, LL2, PL
        lower = np.array([-0.23, 0.210,  0.23,  0.20,  0.04,  0.04,  0.23,  -0.07,  -7,  -7,  -11,  -2, -5], dtype=np.float32)  # 每個維度的最小值
        upper = np.array([-0.21, 0.235,  0.28,  0.26,  0.13,  0.13,  0.29,  -0.03,  -1,  -2,    5,   6, -3], dtype=np.float32)  # 每個維度的最大值

        self.action_space = spaces.Box(low=lower, high=upper) 

    def setheight(self, progress):
        if self.z_CoM >= 0 and self.alter_CoM[progress] < 0: # 0.35 to 0.45
            self.squat_down(inverse=False)
        elif self.z_CoM < 0 and self.alter_CoM[progress] >= 0: # 0.45 to 0.35
            self.squat_down(inverse=True)
        else:
            pass
        self.z_CoM = self.alter_CoM[progress]
        self.obs_height[:] = self.z_CoM

    def squat_down(self, inverse=False):
        self.Robot.squat_down(inverse)

    def reset(self):
        # 重置環境狀態
        print("reset")

        '''reset the np random'''
        # to ensure the random seed
        self.np_random, _ = gym.utils.seeding.np_random()

        self.done = False
        self.fall = False
        self.complete_one = False
        self.stair_porgress = 0 # go to initial stair

        self.setheight(self.stair_porgress)
        self.LIPM_obj = Forward2(self.filename, self.z_CoM)
        self.LIPM_obj.output_motion()

        # noise_low = -self._reset_noise_scale
        # noise_high = self._reset_noise_scale
        # pos_noise = self.np_random.uniform(
        #     low=noise_low, high=noise_high, size=21
        # )
        # vel_noise = self.np_random.uniform(
        #     low=noise_low, high=noise_high, size=28
        # )
        # self.Sim_obj.reset(pos_noise, vel_noise)


        self.flagEndOneStep = True
        self.num = 0

        self.basequat = np.array([0]*4*5, dtype=np.float32)
        self.baseacc = np.array([0]*3*5, dtype=np.float32)
        # basequat, baseacc, leg, vel = self._get_obs()
        self.basequat[4*(5-1):] = self.quat
        self.baseacc[3*(5-1):] = self.acc
        self.obs_height[:] = self.alter_CoM[0]
        obs = np.concatenate((self.basequat, self.baseacc, self.pos, self.vel, self.obs_height), axis=None)
        return obs
    
    def get_action(self, num):
        """In begin of step, Get the LIPM series motion sliced and return
        Args:
            num (int): the n_th group
            action (list): LIPM motion
        """
        if self.flagEndOneStep == True:
            self.flagEndOneStep = False
            self.num = 0

        '''Get the leg_position to return'''
        if self.N*(num+1) > 200:
            # The last motion
            self.flagEndOneStep = True # Set the flag to create another LIPM motion'''
            leg_position=self.action_slice[self.N*num:]
            self.num = 0 # reset the calculated_group
        else:
            leg_position=self.action_slice[self.N*num:self.N*(num+1)]
            self.num += 1 # calculate the group
        
        return leg_position
    
    def is_healthy(self):
        """Check the robot is healthy or not

        Returns:
            bool: fall or not
        """
        is_healthy = True
        base_roll, base_pitch, base_yaw = quart2rpy(self.quat)
        if base_pitch < -0.785 or base_pitch > 0.785 or base_roll < -0.785 or base_roll > 0.785 \
            or not (-0.5 < base_yaw < 0.5):
            is_healthy = False
            print("Action Stop! Out of safe region.")

        return is_healthy

    def outputLIPM(self, action):
        self.LIPM_obj.setParameter(action, self.z_CoM)
        self.LIPM_obj.output_motion()

        self.Robot.set_motion()


    def step(self, action): 
        lipm = threading.Thread(target=self.outputLIPM, args=(action,))
        lipm.start()

        slice_motion = self.get_action(self.num)
        # print(slice_motion)
        obs = []
        reward = 0

        # 執行一個行動，返回下一步觀察、獎勵、終止標誌和額外信息
        for i in range(len(slice_motion)): 
            flag = self.Robot.go_sim2(slice_motion[i])
            if flag:
                flag = self.is_healthy()
            # basequat, baseacc, leg, vel = self._get_obs()

            if i < 5:
                self.basequat[i*4:i*4+4] = self.quat
                self.baseacc[i*3:i*3+3] = self.acc
                obs = np.concatenate((self.basequat, self.baseacc, self.pos, self.vel, self.obs_height), axis=None)

            if flag == 0 or flag == 3:
                # print("FAIL")
                self.done = True
                self.fall = True
                reward -= 20
                break

            elif i == 149:
                reward += 20
                self.done = False
                self.complete_one = True
                print("complete: ", self.stair_porgress)
                # break
        lipm.join()

        info = {
            "reward": reward,
            "reward_alive": self.fall,
            "complete": self.complete_one
        }

        if self.complete_one:
            if self.stair_porgress == len(self.alter_CoM)-1:
                self.done = True
                reward+=200
                print("Pass all!")
            else:
                self.complete_one=0
                self.stair_porgress+=1 # Enter next staircase
                # self.setheight(self.stair_porgress)

        return obs, reward, self.done, info
    
    def set_obs(self, base_info, base_acc, motor_pos, motor_vel):
        self.quat = base_info
        self.acc = base_acc
        self.pos = motor_pos
        self.vel = motor_vel

    
    def _get_obs(self):                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        basequat, baseacc, leg, leg_vel = self.Robot.real_obs()

        return basequat, baseacc, leg, leg_vel
    
    
    def Cal_square_err(self, a, b):
        return (a-b)**2
    

    def render(self, mode="human"):
        # 渲染環境，這裡可以是圖形界面或其他方式
        pass

    def close(self):
        # 關閉環境，釋放資源
        pass

