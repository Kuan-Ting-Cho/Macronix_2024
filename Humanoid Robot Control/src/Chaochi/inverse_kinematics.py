import numpy as np
from math import sin,cos,pi

def com_sin_trace(t,am,T):
    [am_x,am_y,am_z]=am
    x=am_x/2+am_x/2*sin(t*2*pi/T-pi/2)
    y=am_y/2+am_y/2*sin(t*2*pi/T-pi/2)
    z=am_z/2+am_z/2*sin(t*2*pi/T-pi/2)
    return np.array([x,y,z])

def com_circle_trace(t,r,T,h):
    x=r*sin(t*2*pi/T)
    y=r*cos(t*2*pi/T)
    return np.array([x,y,h])

def InvK_CHI(P,R=[[1, 0, 0],[0, 1, 0],[0, 0, 1]]):
    [Px,Py,Pz] = P
    # for i in range(3):
    #     for j in range(3):
    #         if abs(R[i][j]) < 0.05:
    #             R[i][j]=0
    [r11, r12, r13] = R[0]
    [r21, r22, r23] = R[1]
    [r31, r32, r33] = R[2]

    D1, D2, D3= -0.064 ,0.006 ,-0.00725
    [L2, L3, L4, L5, L6] = [0.102, 0.35795, 0.36642, 0.029, 0.11175]

    #theta2
    a = Pz + L6 * r33 - D1
    b = Py + L6 * r23
    r = (a**2 + b**2)**(1/2)
    t2 = -np.arctan2(b, a) + np.arctan2(D3/r, -(1 - (D3/r))**(1/2))
    while t2 >pi:
        t2-=2*pi
    while t2 <-pi:
        t2+=2*pi
        
    #theta6
    t6 = np.arcsin(-r23 * np.cos(t2) - r33 * np.sin(t2))

    #theta4
    H04x = Px + r13 * (L6 + L5 * np.cos(t6)) + L5 * r12 * np.sin(t6)
    H04y = Py + r23 * (L6 + L5 * np.cos(t6)) + L5 * r22 * np.sin(t6)
    H04z = Pz + r33 * (L6 + L5 * np.cos(t6)) + L5 * r32 * np.sin(t6)

    H02x = D2
    H02y = L2 * np.sin(t2)
    H02z = D1 - L2 * np.cos(t2)

    H04 = [H04x, H04y, H04z] #Motor5 position-
    H02 = [H02x, H02y, H02z] #Motor3 position

    L35 = np.sqrt((H04[0] - H02[0])**2 + (H04[1] - H02[1])**2 +  #L35
                    (H04[2] - H02[2])**2)
    L35 = L35 * np.sin(np.arccos(0.00725/L35))
    l3 = 0.35795
    l4 = 0.36642
    theta = (l3**2 + l4**2 - L35**2) / (2 * l3 * l4) #theta4

    if theta > 1:
        theta = 1
    elif theta < -1:
        theta = -1
    else:
        theta = theta
    t4 = np.pi - np.arccos(theta) - np.arccos(0.366/0.36642)

    c4 = np.cos(t4 + np.pi*2.74/180)
    s4 = (1-c4**2)**(1/2)

    #theta3
    alpha = D1 * np.cos(t2) + Py * np.sin(t2) - Pz * np.cos(t2) - (L6 + L5 * np.cos(
        t6)) * (r33 * np.cos(t2) - r23 * np.sin(t2)) - L2 - L5 * np.sin(
            t6) * (r32 * np.cos(t2) - r22 * np.sin(t2))  #B
    beta = D2 - Px - r13 * (L6 + L5 * np.cos(t6)) - L5 * r12 * np.sin(t6) #A
    gamma = L3 + L4 * c4
    phi = L3 * gamma + L4**2 * s4**2 + L4 * c4 * gamma #sin(theta3) 的分母

    test_sin = (beta * gamma - alpha * L4 * s4) / phi 
    if (test_sin > 1 or test_sin < -1):
        print("theta 3 error !")

    t3 = np.arcsin((beta * gamma - alpha * L4 * s4) / phi ) #theta3
    #theta5
    A = np.cos(t6) * (r33 * np.cos(t2) - r23 * np.sin(t2)) + np.sin(
        t6) * (r32 * np.cos(t2) - r22 * np.sin(t2)) #C
    B = r13 * np.cos(t6) + r12 * np.sin(t6) #D

    t5 = np.arcsin(B * np.cos(t3 + t4 + np.pi*2.74/180) - A * np.sin(t3 + t4 + np.pi*2.74/180)) + np.pi*2.74/180 #theta5
    return list(np.array([0,t2,t3,t4,t5,t6])*180/pi)