import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

k = pd.read_csv("inertialOnlyTrajectory.txt")

fig = plt.figure(figsize = (16, 9))
ax = plt.axes(projection ="3d")
ax.scatter3D(k.x, k.y, k.z)


# plt.scatter(k.x, k.y)
plt.show()