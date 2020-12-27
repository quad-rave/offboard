import numpy as np
import math
import time

class TwoWayDict(object):
    def __init__(self):
        self.key_to_value = {}
        self.value_to_key = {}
    
    def add(self, key, value):
        self.key_to_value[key] = value
        self.value_to_key[value] = key

    def get_value(self, key):
        return self.key_to_value[key]
    def get_key(self, value):
        return self.value_to_key[value]

def dist(a,b):
    return np.linalg.norm(a - b)


def vec(x,y,z):
    return np.array([x,y,z])

'''
0 0 -1
0 1 0
1 0 0

'''
'''
R'(psi_1) = (R(psi_1) - R(psi_0)) / (psi_1 - psi_0)
u_f = R(psi_1) * target_delta - current_delta # - u_c
u_c = R'(psi_1) * psi_1' * target_delta

psi_1 = pi/2 = psi(t_i + t_m)
t_i = start_time
t_mi = elapsed_time

gamma = { (4 * (psi_1 - psi(t_i)) / t_f^2, 
          (-4 * (psi_1 - psi(t_i)) / t_f^2,
          0
}
#psi(t_i) == psi
#psi(t_i + t_mi) == psi + someValue

'''

def rotation_matrix(psi, axis='Z'):
    mat = np.eye(3)
    if axis=="X":
        mat[1,1] = math.cos(psi)
        mat[1,2] = -math.sin(psi)
        mat[2,1] = math.sin(psi)
        mat[2,2] = math.cos(psi)
    elif axis == "Y":
        mat[0,0] = math.cos(psi)
        mat[0,2] = math.sin(psi)
        mat[2,0] = -math.sin(psi)
        mat[2,2] = math.cos(psi)
    elif axis == "Z":
        mat[0,0] = math.cos(psi)
        mat[0,1] = -math.sin(psi) 
        mat[1,0] = math.sin(psi) 
        mat[1,1] = math.cos(psi)
    
    return mat

def rotation_matrix_derivate(psi, axis='Z'):
    mat = np.eye(3)
    if axis=="X":
        mat[1,1] = -math.sin(psi)
        mat[1,2] = -math.cos(psi)
        mat[2,1] = math.cos(psi)
        mat[2,2] = -math.sin(psi)
    elif axis == "Y":
        mat[0,0] = -math.sin(psi)
        mat[0,2] = math.cos(psi)
        mat[2,0] = -math.cos(psi)
        mat[2,2] = -math.sin(psi)
    elif axis == "Z":
        mat[0,0] = -math.sin(psi)
        mat[0,1] = math.cos(psi) 
        mat[1,0] = math.cos(psi) 
        mat[1,1] = -math.sin(psi)
    
    return mat
def lerp(start, end, interval):
    return (end - start) * interval + start
def inverse_lerp(start,end, value):
    return (value - start) / (end - start)
def remap(a, a0, a1, b0, b1):
    interval = inverse_lerp(a0,a1,a)
    return lerp(b0,b1, interval)


if __name__ == "__main__":
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    print("STARTLOOP-STARTLOOP-STARTLOOP-STARTLOOP-STARTLOOP-STARTLOOP-STARTLOOP-STARTLOOP")
    loopvec = rotation_matrix(0.0, "Z")
    loopmat = rotation_matrix(2*math.pi * 0.5 * (1.0/100.0))
    for i in range(100):
        print(loopvec)
        print("mag: ", np.linalg.norm(loopvec))
        loopvec = np.matmul(loopmat, loopvec)
        time.sleep(0.2)

    #print_mat(result)
    