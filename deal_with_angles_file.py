import sys

filename = sys.argv[1]

def reorder(rowlist):
    n = len(rowlist)
    for i in range(0,n,2):
        if(rowlist[i%n] == 'BS1' and rowlist[(i+2)%n] == 'BS1'):
            break
    rowlist = [rowlist[(j+i+1)%n] for j in range(0,n,2)]
    return ' '.join(rowlist)+"\n"   

f = open(filename,"r")
data = f.readlines()

data = [reorder(row.split()) for row in data if len(row.split())==8]

ff = open(filename[:-4]+"_reorderd.txt","w")

ff.writelines(data)
f.close()
ff.close()

 
