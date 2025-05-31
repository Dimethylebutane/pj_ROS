import numpy as np
import matplotlib.pyplot as plt

def matvec(A, B):
    return np.array([A@b for b in B])


def tilde(P):
    return np.column_stack((P, np.ones(P.shape[0])))

K = np.array([
    [453.49, 0,    320],
    [0,    453.49, 240],
    [0,    0,       1],
    ])


alpha = 0.52
h = 0.11 #m
a = 0.29
sa = np.sin(alpha)
ca = np.cos(alpha)

R = np.linalg.inv(np.array([
            [0,  sa,  ca, a],
            [1,  0,   0,  0],
            [0,  ca, -sa, h],
            [0,  0,   0,  1],
            ]))[:-1]

#P = np.array([h*np.tan(np.pi/2 - alpha), 0, 0])
P = np.array([[h*np.tan(np.pi/2 - alpha), 0, 0], [1, 0, 0], [0.5, 0, 0], [1, 1, 0], [1, 1, 0]])
P = tilde(P)
print("P", P.shape)
#print("P", P)

SU = matvec(K @ R, P)
print("SU", SU.shape)
S = 1/SU[:,-1]
U = np.array(np.multiply(SU[:,:-1], S.reshape((S.shape[0],1))) + 0.5, dtype=np.int32)
print("U, S", U.shape, S.shape)

Cam = np.array([a, 0, h])#position de la camera dans l'espace % pied robot

U = tilde(U)
print("U", U.shape, U)

print(R.shape)
#R = np.linalg.inv(np.array([
#            [0,  sa,  ca, a],
#            [1,  0,   0,  0],
#            [0,  ca, -sa, h],
#            [0,  0,   0,  1],
#            ]))
#R = R@R
#R = R[:-1]


pinv = np.linalg.pinv(K@R)
print("pinv", pinv.shape)
di = matvec(pinv, U)[:,:-1]
print("di", di.shape)

dz = di[:,-1]
t = Cam[-1]/dz
Pest = Cam-np.multiply(t.reshape(t.shape[0], 1), di)
print("pest shape:", Pest, P[:, :-1])
#print("t", t)
print("Pest", 1*(np.linalg.norm((Pest-P[:,:-1]), axis=1)  ) )
