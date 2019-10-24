import numpy as np
import cboids
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


positions = cboids.simulate_boids(50, 2.0, 100, 10000)
for i in range(50):
    xyz = positions[:, i, :3].T
    ax.plot(*xyz)
plt.show()
