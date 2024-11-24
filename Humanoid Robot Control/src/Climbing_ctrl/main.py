import numpy as np
from stable_baselines3 import SAC, TD3
from Enviroment.env5_wo_vel import RolyEnv
from Climbing_ctrl.function import *
from robot.scripts.cmd import *
import threading


status=np.ones((16,4),dtype='float32')
pos=[0.0]*13
vel=[0.0]*13
l_fsr=[0.0]*4
r_fsr=[0.0]*4
IMU_data=[0,0,0]
init_yaw_bias = 0.0



def real_obs():        
    global pos, vel, IMU_data
    base_info = rpy2quart(IMU_data[0], IMU_data[1], IMU_data[2]-init_yaw_bias)
    base_acc = np.array([0]*3)
    motor_pos = np.array(pos) # 13
    motor_vel = np.array(vel)

    return base_info, base_acc, motor_pos, motor_vel



def get_status():
    global status, pos, vel, l_fsr, r_fsr, IMU_data
    while True:
        status, pos, vel=update_status()
        IMU_data=status[-3][:3]
        l_fsr=status[-2]
        r_fsr=status[-1]


def get_alter_z():
    while True:
        try:
            alter_z = int(input("Stair height: "))
            if abs(alter_z) > 10:
                print("輸入錯誤，數字必須小於等於 10請重新輸入。")
            else:
                return alter_z
                
        except ValueError:
            print("請輸入有效的數字。")

def set_env_parameters(env):
    alter_z = get_alter_z()
    env.setheight(alter_z)

def main():
    global init_yaw_bias
    file = "./Climbing_ctrl/model/SAC.zip"
    test_algo = 1

    # 創建自己的環境
    env = RolyEnv(True, "./Climbing_ctrl/motordata/forward_data_test.csv")

    # 使用模型進行預測
    obs = env.reset()
    init_yaw_bias = IMU_data[2]
    q, a, p, v = real_obs()
    env.set_obs(q, a, p, v)

    while True:
        set_env_parameters(env)        

        if test_algo == 1:
            model = SAC.load(file, env=env, device="cuda",custom_objects = {'observation_space': env.observation_space, 'action_space': env.action_space})
        elif test_algo == 3:
            model = TD3.load(file, env=env, device="cuda")

        
        done_one=False
        while not done_one:
            action, _ = model.predict(obs, deterministic=True)
            # print(action)
            q, a, p, v = real_obs()
            env.set_obs(q, a, p, v)
            obs, reward, done, info = env.step(action)
            done_one = info['complete']

            if done: # fall or complete
                obs = env.reset()
                break




if __name__=='__main__':
    activation()
    try:
        imu = threading.Thread(target = update_IMU)
        # fsr = threading.Thread(target = update_fsr)
        recv = threading.Thread(target = get_status)
        cmd = threading.Thread(target = main)
        imu.start()
        # fsr.start()
        recv.start()
        cmd.start()
        imu.join()
        # fsr.join()
        recv.join()
        cmd.join()
        
        print("end")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
