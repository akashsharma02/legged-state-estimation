import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os.path

k = pd.read_csv("Trajectory.txt")

fig = plt.figure(figsize = (16, 9))
ax = plt.axes(projection ="3d")
ax.plot3D(k.x, k.y, k.z, c='r')

gt = pd.read_csv("gt_traj.txt")
ax.plot3D(gt.x, gt.y, gt.z, c='g')

if (os.path.exists("unOptTrajectory.txt")):
    unOpt = pd.read_csv("unOptTrajectory.txt")
    ax.plot3D(unOpt.x, unOpt.y, unOpt.z, c='y')
    ax.legend(['Optimized Result', 'Ground Truth', 'Unoptimized Result'])
else:
    ax.set_zlim(0, 1)
    ax.legend(['Result', 'Ground Truth'])

# plt.scatter(k.x, k.y)
plt.show()
