import sys
import numpy as np
n = int(sys.argv[1])
filename = sys.argv[2]

f = open(filename,"r")
fout = open(filename[:-4]+"_Average.txt","w")
data = f.readlines()
angles = []
for i in range(n+1):
    angles.append([])

for row in data:
    row = row.split()
    for i in range(1,n+1):
        if row[0] == str(i):
            angles[i].append(list(map(float,row[1:])))
            break
for i in range(1,n+1):
    angles[i] = np.mean(np.array(angles[i]),axis=0)

fout.write(str(n)+"\n")
for i in range(1,n+1):
    for angle in angles[i]:
        fout.write(str(angle)+" ")
    fout.write("\n")
f.close()
fout.close()

