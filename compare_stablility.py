import matplotlib.pyplot as plt
import numpy as np
import sys

filename1 = sys.argv[1]
filename2 = sys.argv[2]

f1 = open(filename1,"r")
f2 = open(filename2,"r")

data1 = np.array([list(map(float,row.split())) for row in f1.readlines()])
data2 = np.array([list(map(float,row.split())) for row in f2.readlines()])

p1 = np.mean(data1,axis=0)
p2 = np.mean(data2,axis=0)

data1 -= p1
data2 -= p2

fig = plt.figure()
axs = []
n = 50
for i in range(4):
    axs.append(fig.add_subplot(4,1,i+1))
    y = np.hstack((data1[:n,[i]],data2[:n,[i]]))
    axs[i].plot(y)
    axs[i].legend(['TM4C','Arduino'])
plt.show()
