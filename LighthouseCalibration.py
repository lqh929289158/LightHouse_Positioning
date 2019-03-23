from sympy import sin,cos,Symbol,nsolve
import sympy
import mpmath
import numpy as np
mpmath.mp.dps = -1000

# In the coordinate of Lighthouse
# X: Front
# Y: Left
# Z: Up
# Convert the horizontal/vertical angle to coordinate(x,y,z)
# Input: theta,phi. Type:list
def convert_angle_to_direction_vector(theta,phi):
    #convert list to numpy line vector
    theta = np.array([theta])
    phi = np.array([phi])
    n = theta.shape[1]
    if (phi.shape[1] != n):
        print("theta and phi do not match size")
        return 
    # compute horizontal/vertical scanning planes' normal vector
    # H,V (n,3)
    H = np.hstack((np.cos(theta+np.pi/2).T,np.sin(theta+np.pi/2).T,np.zeros((n,1))))
    V = np.hstack((np.cos(phi+np.pi/2).T,np.zeros((n,1)),np.sin(theta+np.pi/2).T))
    # compute direction vector of every points(line vectors)
    D = np.cross(H,V)
    # Normalize all line vectors
    D = D/(np.array([np.linalg.norm(D,axis=1)]).T)
    return D
# Input: alpha,beta,gamma. Type: Symbol
def get_rotation_matrix_symbol(a,b,r):
    R = np.array([\
    [cos(a)*cos(b)*cos(r)-sin(a)*sin(r),-cos(a)*cos(b)*sin(r)-sin(a)*cos(r),cos(a)*sin(b)],\
    [sin(a)*cos(b)*cos(r)+cos(a)*sin(r),-sin(a)*cos(b)*sin(r)+cos(a)*cos(r),sin(a)*sin(b)],\
    [-sin(b)*cos(r)                    ,sin(b)*sin(r)                      ,cos(b)]\
    ])
    return R

# read angles from file
# Input: file name(type:string) #bs(type:Integer 0~1)
# File format
# line 1: n(number of point)
# line i: BS1 H BS1 V  BS2 H  BS2 V(point i-2)
def read_angle_from_file(filename,bs):
    f = open(filename,"r")
    n = int(f.readline())
    # angles is a 2-D array of shape(n,4)
    angles = np.array([list(map(float,row.split())) for row in f.readlines()])
    # return theta: Base station 1 Horizontal
    theta = list(angles[:,0+bs*2])
    theta = [angle/180*np.pi for angle in theta]
    # return phi  : Base station 1 Vertical
    phi   = list(angles[:,1+bs*2])
    phi   = [angle/180*np.pi for angle in phi]
    f.close()
    return n,theta,phi

# read direction vectors of all calibration points from file
# Input: file name, number of points
# line i: x y z(direction vector from O[O coordinate] to point i
def read_direction_vectors_from_file(filename,n):
    f = open(filename,"r")
    # D is of shape(n,3)
    D = np.array([list(map(float,row.split())) for row in f.readlines()[:n]])
    # D is of shape(3,n)
    D = D.T
    # every column is the vector (from original point[O coordinate] to point i)
    return D
    


n = 5 #Number of point (including one original point)

#(n-1)*3 equations
#n + 3 symbols: p0,p1,...,pn,alpha,beta,gamma

#theta: horizontal angle(-pi/2 ~ pi/2)
#phi: vertical angle(-pi/2 ~ pi/2)
theta = []
phi = []

bs = 0
n,theta,phi = read_angle_from_file('Calibration_Data_TM4C_Average.txt',bs)
# only for 1 Base station, theta is [Horizontal Angle for point 0, Horitontal Angle for point 1..]
# only for 1 Base station, phi   is [Vertical   Angle for point 1, Vertical   Angle for point 1..]

direction_vectors_from_BS = convert_angle_to_direction_vector(theta,phi)
direction_vectors_from_BS = direction_vectors_from_BS.T
# direction_vectors is of shape (3,n)
# every column is the direction vector (from original point[BS coordinate] to point i)

direction_vectors_from_O = read_direction_vectors_from_file('Direction_Vectors.txt',n)
# direction_vectors_from_O is of shape(3,n)
# every column is the direction vector (from original point[O coordinate] to point i)

p = []
# parameter p0,...,pn-1: Distance from Lighthouse to point0,...point n-1
for i in range(n):
    p.append(Symbol('p'+str(i)))
# parameter a,b,r in rotation matrix
alpha = Symbol('alpha')
beta = Symbol('beta')
gamma = Symbol('gamma')

# rotation matrix
R = get_rotation_matrix_symbol(alpha,beta,gamma)

equation_system = []
# list 3 equations for each point 1,...,n-1
for i in range(1,n):
    f = p[0]*direction_vectors_from_BS[:,[0]]+R*direction_vectors_from_O[:,[i]]\
       -p[i]*direction_vectors_from_BS[:,[i]]
    # f is a column vector (3,1), containing 3 equations for point i
    # append all equations to equations list
    equation_system += list(f.flatten())



# organize symbols

symbol_vector = []
symbol_estimate = [2.2]*n
if bs == 0:
    symbol_estimate += [-np.pi/12+0.001j,-np.pi/12+0.001j,-np.pi/4+0.001j]
elif bs == 1:
    symbol_estimate += [-np.pi/4,-np.pi/12,3/4*np.pi]
symbol_vector += p
symbol_vector += [alpha,beta,gamma]

# convert all lists to tuples
equation_system = tuple(equation_system)
symbol_vector = tuple(symbol_vector)
symbol_estimate = tuple(symbol_estimate)
print(nsolve(equation_system,symbol_vector,symbol_estimate,prec=1))
