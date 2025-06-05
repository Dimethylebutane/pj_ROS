import time
import matplotlib.pyplot as plt
import numpy as np
plt.ion()
x = np.arange(0, 4*np.pi, 0.1)
y = [np.sin(i) for i in x]
plt.plot(x, y, 'g-', linewidth=1.5, markersize=4)
plt.pause(0.)         
plt.plot(x, [i**2 for i in y], 'g-', linewidth=1.5, markersize=4)
plt.pause(0.)
plt.plot(x, [i**2*i+0.25 for i in y], 'r-', linewidth=1.5, markersize=4) 
plt.pause(0.)
