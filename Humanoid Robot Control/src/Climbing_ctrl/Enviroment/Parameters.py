import numpy as np

# 定義不同類型的參數
upstairs = {
    "lower": np.array([-0.22, 0.21, 0.23, 0.22, 0.10, 0.10, 0.23, -7, -7, -2, -2, -5], dtype=np.float32),
    "upper": np.array([-0.21, 0.22, 0.27, 0.26, 0.13, 0.13, 0.29, -4, -4,  5,  5, -3], dtype=np.float32)
}
downstairs = {
    "lower": np.array([-0.22, 0.225, 0.24, 0.22, 0.10, 0.10, 0.26, -0.07, -4, -5, -11, 4], dtype=np.float32),
    "upper": np.array([-0.21, 0.235, 0.28, 0.26, 0.13, 0.13, 0.30, -0.03, -2, -3, -9, 6], dtype=np.float32)
}
plane = {
    "lower": np.array([-0.23, 0.21, 0.24, 0.20, 0.04, 0.04, 0.13, -3, -4, 1, 2], dtype=np.float32),
    "upper": np.array([-0.22, 0.22, 0.28, 0.22, 0.07, 0.07, 0.17, -1, -2, 3, 4], dtype=np.float32)
}

