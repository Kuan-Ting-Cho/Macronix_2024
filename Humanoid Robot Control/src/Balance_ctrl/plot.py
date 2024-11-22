import matplotlib.pyplot as plt
import csv
import numpy as np
file = open('localcom/Chaochi/zmp.csv')
reader = csv.reader(file)
w = list(reader)
lx = [float(i) for i in w[0]]
ly = [float(i) for i in w[1]]
rx = [float(i) for i in w[2]]
ry = [float(i) for i in w[3]]
file.close()
zx=[(lx[i]+rx[i])/2 for i in range(len(lx))]
zy=[(ly[i]+ry[i])/2 for i in range(len(lx))]
x = np.linspace(1, len(lx), len(lx))
plt.plot(x,zx,'r')
plt.plot(x,zy,'g')
# plt.plot(x,rx,'b')
# plt.plot(x,ry)
plt.legend([r'Z$_{l,x}$',r'Z$_{l,y}$',r'Z$_{r,x}$',r'Z$_{r,y}$'])
plt.xlabel('Timestep (1 step = 0.02s)')
plt.ylabel('Zmp_pos (mm)')
plt.show()