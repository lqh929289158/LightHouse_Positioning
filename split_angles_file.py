import sys

num = int(sys.argv[1])
filename = sys.argv[2]

f = open(filename,"r")

flist = [0]
for i in range(1,num+1):
    flist.append(open(filename[:-4]+"_"+str(i)+".txt","w"))

data = f.readlines()
f.close()
for row in data:
    row = row.split()
    for i in range(1,num+1):
        if row[0] == str(i):
            for item in row[1:]:
                flist[i].write(item+" ")
            flist[i].write("\n")
            continue
for ff in flist[1:]:
    ff.close()
