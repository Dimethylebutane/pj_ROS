import numpy as np
import matplotlib.pyplot as plt

def tilde(P):
    return np.append(P, np.ones((P.shape[0],1)), axis=1)

K = np.array([
    [1000000, 0,    240],
    [0,    1000000, 320],
    [0,    0,       1],
    ])
alpha = 0.52
h = 0.11 #m
sa = np.sin(alpha)
ca = np.cos(alpha)

R = np.linalg.inv(np.array([
            [0,  sa,  ca, 0],
            [1,  0,   0,  0],
            [0,  ca, -sa, h],
            [0,  0,   0,  1],
            ]))[:-1]

#P = np.array([h*np.tan(np.pi/2 - alpha), 0, 0])
P = np.array([[1, 1, 0]])
P = tilde(P)
print("P", P, P[0])


SU = K @ R @ P[0]
print("SU", SU)
S = SU[-1]
U = SU[:-1] / S
print("U, S", U, S)

SU = np.matvec(K @ R, P)
print("SU", SU)
S = SU[:,-1]
U = SU[:,:-1] / S
print("U, S", U, S)
exit()


