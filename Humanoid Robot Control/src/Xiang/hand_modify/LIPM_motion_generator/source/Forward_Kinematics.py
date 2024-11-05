import math
import numpy as np
import matplotlib.pyplot as plt
import os

def single_transfomation_matrix(theta, d, a, alpha):
    result = [[ math.cos(theta), -math.cos(alpha)*math.sin(theta),  math.sin(alpha)*math.sin(theta),     a*math.cos(theta)],
              [	math.sin(theta),  math.cos(alpha)*math.cos(theta), -math.sin(alpha)*math.cos(theta),     a*math.sin(theta)],
              [               0,	              math.sin(alpha),	                math.cos(alpha),                    d ],
              [               0,	                            0,	                              0,                    1 ]]
    
    return np.array(result)

""" 
    @brief  all_transfomation_matrix : Calculate transfomation matrix

    @param  theta - motor angle in z direction
    @param  d - leg length in z direction
    @param  a - leg length in x direction
    @param  alpha - motor angle in x direction
"""
def all_transfomation_matrix(theta, d, a, alpha):
    t5e = [[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]]
    for i in range(len(theta)):
        single_matrix=single_transfomation_matrix(theta[i], d[i], a[i], alpha[i])
        if i == 0:
            all_matrix=single_matrix
        else:
            all_matrix=all_matrix.dot(single_matrix)

        all_matrix=all_matrix.dot(t5e)

    return all_matrix

def main():
    theta=np.array([90, 0, -90, 0, 177.26, -177.26])
    d=np.array([0, 64, 6, 0, 0, 7.25])
    a=np.array([105, 0, 102, 357.95, 366.42, 29])
    alpha=np.array([0, 90, 90, 0, 0, -90])
    result=all_transfomation_matrix(theta,d,a,alpha)
    #print(result)
    print(result[0:3,3])

if __name__=='__main__':
    main()


