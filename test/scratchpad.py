import numpy as np
import matplotlib.pyplot as plt

q = np.array([0.0017455732073894737, -0.0059009600595707565, 0.27540183129500356, 0.9613095042513164])
a = np.arccos(q[-1])
print(a*180/np.pi)
