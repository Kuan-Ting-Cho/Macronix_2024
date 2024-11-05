from scipy.stats import norm
#print每顆馬達的角度圖
for i in range(1,13):
    plt.subplot(2, 6, i)
    motion[i].plot()
    if  i==4:
        point,y=generate_normal_distribution(motion[i],balance_step,30)
        motion[i][point[0]:point[2]] = y+0.7
        motion[i].plot()
    if  i==10:
        point,y=generate_normal_distribution(motion[i],balance_step,30)
        motion[i][point[0]:point[2]] = y+0.7
        motion[i].plot()
    plt.xlabel('step')
    plt.ylabel('radius')
    plt.title(joint_name[i-1])
plt.show()

# 尋找關節step突跳點
def find_inflection_points_and_max_value(data,balance_step):
    # 查找转折点
    inflection_points = []
    print(len(data))
    for i in range(1, len(data) - 1):
        if i>30:
            if (data[i] < data[i - 1] and data[i] < data[i + 1]) or (data[i] > data[i - 1] and data[i] > data[i + 1]):
                inflection_points.append(i)
            elif data[i] == data[i - 1] and (data[i] > data[i + 1] or data[i] < data[i + 1]):
                inflection_points.append(i)
            elif (data[i] > data[i - 1] or data[i] < data[i - 1]) and data[i] == data[i + 1]:
                inflection_points.append(i)
    center_value = data[inflection_points[1]]-data[inflection_points[0]]
    inflection_points=[i+balance_step for i in inflection_points] # balance_step=站直與初始蹲姿間插值的step
    return inflection_points, center_value

def generate_normal_distribution(motion,balance_step,extend_step): 
    point,center_value=find_inflection_points_and_max_value(motion[balance_step:].tolist(),balance_step)
    print(point,center_value)
    step=point[2]-point[0] #step=突波的step
    # 生成正態分布數據
    mean=0
    std_dev=1
    x = np.linspace(mean - 3 * std_dev, mean + 3 * std_dev, step+extend_step)
    pdf = norm.pdf(x, loc=mean, scale=std_dev)
    # 根據中間值縮放數據
    scaled_pdf=pdf*center_value / max(pdf)
    # 將轉折點頭尾擴大
    point[0]=point[0]-int(extend_step/2)
    point[2]=point[2]+int(extend_step/2)
    return point,scaled_pdf