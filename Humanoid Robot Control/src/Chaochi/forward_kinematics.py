import numpy as np
import math
from scipy.spatial.transform import Rotation as R
pi = math.pi


def deg2rad(angle):
    global pi
    return angle*pi/180

def rad2deg(radius):
    global pi
    return radius/pi*180

def eulerAngles2rotationMat(theta, format='degree'):
    """
    Calculates Rotation Matrix given euler angles.
    :param theta: 1-by-3 list [rx, ry, rz] angle in degree
    :return:
    """
    if format == 'degree':
        theta = [i * math.pi / 180.0 for i in theta]
 
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
 
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
 
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def isRotationMatrix(R):
    # Checks if a matrix is a valid rotation matrix.
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])
 

def forward_k(x):
    x1 = x[0]
    x2 = x[1]
    x3 = x[2]
    x4 = x[3] + 2.74*pi/180
    x5 = x[4] - 2.74*pi/180
    x6 = x[5]
    d1 = -0.064
    d2 = 0.006 
    d3 = -0.00725
    a2 = 0.102
    a3 = 0.35795 
    a4 = 0.36642 
    a5 = 0.029
    a6 = 0.11175
    H01 = np.array([[np.cos(np.pi/2+x1), 0, np.sin(np.pi/2+x1), 0], [np.sin(np.pi/2+x1), 0, -np.cos(np.pi/2+x1), 0], [0, 1, 0, d1], [0, 0, 0, 1]])
    H12 = np.array([[np.sin(x2), 0, np.cos(x2), a2*np.sin(x2)], [-np.cos(x2), 0, np.sin(x2), -a2*np.cos(x2)], [0, -1, 0, d2], [0, 0, 0, 1]])
    H23 = np.array([[np.cos(x3), -np.sin(x3), 0, a3*np.cos(x3)], [np.sin(x3), np.cos(x3), 0, a3*np.sin(x3)], [ 0, 0, 1, d3], [0, 0, 0, 1]])
    H34 = np.array([[np.cos(x4), -np.sin(x4), 0, a4*np.cos(x4)], [np.sin(x4), np.cos(x4), 0, a4*np.sin(x4)], [ 0, 0, 1, 0], [0, 0, 0, 1]])
    H45 = np.array([[np.cos(x5), 0, np.sin(x5), a5*np.cos(x5)], [np.sin(x5), 0, -np.cos(x5), a5*np.sin(x5)], [0, 1, 0, 0], [0, 0, 0, 1]])
    H56 = np.array([[np.cos(x6), -np.sin(x6), 0, a6*np.cos(x6)], [np.sin(x6), np.cos(x6), 0, a6*np.sin(x6)], [0, 0, 1, 0], [0, 0, 0, 1]])
    H6E = np.array([[0, 0, -1, 0], [ 0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    H5E = np.dot(H56, H6E)
    H4E = np.dot(H45, H5E)
    H3E = np.dot(H34, H4E)
    H2E = np.dot(H23, H3E)
    H1E = np.dot(H12, H2E)
    H0E = np.dot(H01, H1E)
    #print(H0E)
    return H0E
    
    # H0E_rotation = np.array([[H0E[0][0], H0E[0][1], H0E[0][2]], [H0E[1][0], H0E[1][1], H0E[1][2]], [H0E[2][0], H0E[2][1], H0E[2][2]]])

    #-----------------------------------------------ref-------------------------------#
    # # 定義原始尤拉角 (roll, pitch, yaw)
    # original_euler = np.array([0.0, 0.4, 0.0])
    # original_euler2 = np.array([0.5, 0.0, 0.0])
    # # 將尤拉角轉換成旋轉矩陣
    # rotation_matrix = R.from_euler('xyz', original_euler).as_matrix()
    # rotation_matrix2 = R.from_euler('xyz', original_euler2).as_matrix()
    # Ro = np.dot(rotation_matrix2, rotation_matrix)
    # new_euler = R.from_matrix(Ro).as_euler('xyz')
    #-----------------------------------------------ref-------------------------------#
    
    # 定義原始尤拉角 (roll, pitch, yaw)
    # print(new_euler)
    # print(rotationMatrixToEulerAngles(Ro))
    # a_euler = quaternion
    # a_rotation = R.from_euler('xyz', a_euler).as_matrix()
    # b = np.dot(H0E_rotation, a_rotation)
    # c = R.from_matrix(b).as_euler('xyz')
    # P = np.array([H0E[0][3],H0E[1][3],H0E[2][3]])  
    
    