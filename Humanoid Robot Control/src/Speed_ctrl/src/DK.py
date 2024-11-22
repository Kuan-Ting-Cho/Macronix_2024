import numpy as np
def deg2rad(angle):
    return angle*np.pi/180

def rad2deg(radius):
    return radius/np.pi*180

def DK(motor,R,foot): #foot 1/2 = Right/Left
    T = np.block([[R, np.zeros((3, 1))], [np.zeros((1, 3)), 1]])
    x1=motor[0]
    x2=motor[1]
    x3=motor[2]
    x4=motor[3]
    x5=motor[4]
    x6=motor[5]
    d1, d2, d3= -0.064 ,0.006 ,-0.00725
    a2, a3,a4,a5,a6 = 0.102, 0.35795,0.36642,0.029,0.11175 #Roli
    H01 = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,d1],[0,0,0,1]])
    H12 = np.array([[np.sin(x2),0,np.cos(x2),a2*np.sin(x2)],[-np.cos(x2),0,np.sin(x2),-a2*np.cos(x2)],[0,-1,0,d2],[0,0,0,1]])
    H23 = np.array([[np.cos(x3),-np.sin(x3),0,a3*np.cos(x3)],[np.sin(x3),np.cos(x3),0,a3*np.sin(x3)],[0,0,1,d3],[0,0,0,1]])
    H34 = np.array([[np.cos(x4),-np.sin(x4),0,a4*np.cos(x4)],[np.sin(x4),np.cos(x4),0,a4*np.sin(x4)],[0,0,1,0],[0,0,0,1]])
    H45 = np.array([[np.cos(x5),0,np.sin(x5),a5*np.cos(x5)],[np.sin(x5),0,-np.cos(x5),a5*np.sin(x5)],[0,1,0,0],[0,0,0,1]])
    H56 = np.array([[np.cos(x6),-np.sin(x6),0,a6*np.cos(x6)],[np.sin(x6),np.cos(x6),0,a6*np.sin(x6)],[0,0,1,0],[0,0,0,1]])
    H6E = T
    
    H5E = np.dot(H56,H6E)
    H4E = np.dot(H45,H5E)
    H3E = np.dot(H34,H4E)
    H2E = np.dot(H23,H3E)
    H1E = np.dot(H12,H2E)
    H0E = np.dot(H01,H1E)

    end_effector=[]
    for i in range(3): #Roli
       end_effector.append(H0E[i][3])

    if foot==1: #右腳
       end_effector[1]+=0.105  #轉成相對於base
    elif foot==2:
       end_effector[1]-=0.105
    
   
    return end_effector