import numpy as np
import pandas as pd
import csv
# 將弧度換算角度進行控制運算
d_motor=[]
def rad2deg(radius):
    return radius/np.pi*180
def csv2cmd(filename):
    file = open(filename)
    reader = csv.reader(file)
    data_list = list(reader)
    file.close()
    for i in range(len(data_list)):
        for j in range(len(data_list[i])):
            data_list[i][j] = float(data_list[i][j])
        data_list[i].insert(0,float(0)) #加入trunk角度
    return data_list

def Walk(step, desired, motion):
    global d_motor
    if step < len(motion):
        for i in range(len(motion[step])):
            desired[i] = rad2deg(motion[step][i])
    return desired
