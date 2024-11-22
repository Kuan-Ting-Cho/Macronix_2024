import gym
from gym import spaces
from LIPM.F3 import Forward2
from Simulate import Simulation
import math
import numpy as np

global PI
PI = math.pi


class RolyEnv(gym.Env):
    def __init__(self, render=False, file=None):
        super(RolyEnv, self).__init__()
        self.render = render
        self.filename = file
        '''Declare a np_random'''
        self.np_random = None
        self.done = False
        self.fall = False

        self._reset_noise_scale = 1e-2

        self.x = np.array([-0.225, 0.18, 0.26, 0.23, 0.1, 0.1, 0.15, 0.015, 0, 0, 0, 0], dtype=np.float32)
        self.Sim_obj = Simulation(self.render)
        self.Sim_obj.go_init()

        # observation_space 表示在一個step後可觀察到的資訊( Ex.觀察機器人質心位置(x, y, z) = 3 dims)。
        pos_low = np.array([-5] * 3, dtype=np.float32)  # base position
        pos_high = np.array([5] * 3, dtype=np.float32)

        foot_zpos_low = np.array([-1] * 2, dtype=np.float32)  # right & left foot z position
        foot_zpos_high = np.array([1] * 2, dtype=np.float32)

        motor_low = np.array([-np.inf] * 82, dtype=np.float32) # pos 28, vel 27
        motor_high = np.array([np.inf] * 82, dtype=np.float32)
        self.observation_space = spaces.Box(
            #                   Base pos,  Foot z position,      motor_status
            #                   x,y,z,w,     x_d, y_d, z_d,
            low=np.concatenate((pos_low, foot_zpos_low, motor_low), axis=0),
            high=np.concatenate((pos_high, foot_zpos_high, motor_high), axis=0),
        )

        lower = np.array([-0.23, 0.21,  0.2,  0.2,  0.07,  0.07,  0.23,      0,  -15, -15, -15, -15], dtype=np.float32)  # 每個維度的最小值
        upper = np.array([-0.21, 0.23,  0.3,  0.3,  0.12,  0.11,  0.35,   0.10,    0,   0,   5,   0], dtype=np.float32)  # 每個維度的最大值

                        #    B1,   B2,   h1,   h2,    H1,    H2,    S1,  Acomz,  CR1, CR2, CL1, CL2
        # lower = np.array([-0.23, 0.13,    0,    0,  0.05,  0.05,  0.35,      0,   -5,  -5,  -5, -5], dtype=np.float32)  # 每個維度的最小值
        # upper = np.array([-0.13, 0.23,  0.6,  0.6,  0.13,  0.13,  0.45,   0.05,    5,   5,   5,  5], dtype=np.float32)  # 每個維度的最大值
        self.action_space = spaces.Box(low=lower, high=upper) 

    def reset(self):
        # 重置環境狀態

        '''reset the np random'''
        # to ensure the random seed
        self.np_random, _ = gym.utils.seeding.np_random()

        self.done = False
        self.fall = False


        self.LIPM_obj = Forward2(self.filename)
        self.LIPM_obj.output_motion()

        noise_low = -self._reset_noise_scale
        noise_high = self._reset_noise_scale
        pos_noise = self.np_random.uniform(
            low=noise_low, high=noise_high, size=21
        )
        vel_noise = self.np_random.uniform(
            low=noise_low, high=noise_high, size=28
        )
        self.Sim_obj.reset(pos_noise, vel_noise)
        # self.Sim_obj.go_sim1()

        obs = self._get_obs()

        return obs

    def step(self, action): 
        self.LIPM_obj.set_parameter(action)
        jointPoses, _ = self.LIPM_obj.GenerateMotionData()
        PRz, PLz = self.LIPM_obj.get_foot_zpos()

        self.Sim_obj.set_motion()
        motion = self.Sim_obj.get_motiom()
        self.Sim_obj.go_sim1()
        
        reward = 0

        # 執行一個行動，返回下一步觀察、獎勵、終止標誌和額外信息
        for i in range(130, len(motion.iloc[:, 0])-1):  # 460 = 100 + 30 + 200 + 100 + 30
            flag, cop_err, x_pos, y_pos, Rz, Lz = self.Sim_obj.go_sim2(i)
            obs = self._get_obs()
            if i < 330:
                z_err = self.Cal_square_err(PRz[i - 130], Rz) + self.Cal_square_err(PLz[i - 130], Lz) # z_err is foot z tracking err
            else:
                z_err = 0

            reward += (-cop_err * 2 + x_pos - abs(y_pos)) + 1 # 1 is healthy reward

            if flag == 0 or flag == 3:
                # print("NOW:     ", i)
                # print("fall", i, "   :", reward)
                self.done = True
                self.fall = True
                reward -= 10
                if i >= 230:
                    reward += (i-230)*1.5
                if i >= 330:
                    reward += (i-330)*3
                break

        info = {
            "reward": reward,
            "reward_alive": self.fall,
            # "x_position": xy_position_after[0],
            # "y_position": xy_position_after[1],
            # "distance_from_origin": np.linalg.norm(xy_position_after, ord=2),
            # "x_velocity": x_velocity,
            # "y_velocity": y_velocity,
            "forward_reward": x_pos*1.5,
        }

        if not self.fall:
            reward += 40
            reward += (460-230)*1.5 + (460-330)*3
            print("PASS: ", reward)

        # print("REward: ", reward)

        return obs, reward, self.done, info
    
    def _get_obs(self):                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        '''Get Leg Pos'''
        body_info = self.Sim_obj.get_obs()
        joint_info = self.Sim_obj.get_motor_status()

        ''' Get Observation to return'''
        observe = np.concatenate((body_info, joint_info),axis=None)

        return observe
    
    
    def Cal_square_err(self, a, b):
        return (a-b)**2
    

    def render(self, mode="human"):
        # 渲染環境，這裡可以是圖形界面或其他方式
        pass

    def close(self):
        # 關閉環境，釋放資源
        pass

