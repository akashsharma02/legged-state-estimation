import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

def plot_data(path, dpath, vpath):
    k = pd.read_csv(path)
    pose = k[['x', 'y', 'z']]
    subsampled_row = pose.iloc[::50]
    fig = plt.figure()
    ax = fig.add_subplot()
    # ax = fig.add_subplot(projection='3d')
    ax.plot(subsampled_row.x, subsampled_row.y)

    d = pd.read_csv(dpath)
    # pose = k[['x', 'y', 'z']]
    # subsampled_row_alter = pose.iloc[::1]
    ax.plot(d.x, d.y)

    v = pd.read_csv(vpath)
    # pose = k[['x', 'y', 'z']]
    # subsampled_row_alter = pose.iloc[::1]
    ax.plot(v.x, v.y)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    # ax.set_zlabel('Z (m)')
    ax.legend(['Ground Truth', 'VIO Only', 'VIO w/ Leg'])
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Awesome Visual SLAM")
    parser.add_argument("-p",
                        "--path",
                        default=None,
                        help="Path to the data file",
                        type=str)
    parser.add_argument("-d",
                        "--dpath",
                        default=None,
                        help="Path to the data file",
                        type=str)
    parser.add_argument("-v",
                        "--vpath",
                        default=None,
                        help="Path to the data file",
                        type=str)
    args = parser.parse_args()
    plot_data(args.path, args.dpath, args.vpath)
