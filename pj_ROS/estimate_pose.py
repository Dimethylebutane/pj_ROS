import numpy as np

def tilde(P):
    return np.column_stack((P, np.ones(P.shape[0])))

def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

matvec = lambda A, B : np.array([A@b for b in B])

def clamp(m, a, M):
    return min(max(m, a), M)


def estimatePose(uvs, KRTi, camPos, height):
    U = tilde(uvs)
    pinv = KRTi
    di = matvec(pinv, U)[:,:-1]
    
    dz = di[:,-1]
    t = height/dz #taille du vecteur pour toucher le sol (z=0)
    Pest = camPos - np.multiply(t.reshape(t.shape[0], 1), di) #point = position camera + vecteur direction*taille
    return Pest


