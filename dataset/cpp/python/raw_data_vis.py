import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

def plot_data(path):
    k = pd.read_csv(path)
    pose = k[['x', 'y', 'z']]
    subsampled_row = pose.iloc[::1]
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter3D(subsampled_row.x, subsampled_row.y, subsampled_row.z)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Awesome Visual SLAM")
    parser.add_argument("-p",
                        "--path",
                        default=None,
                        help="Path to the data file",
                        type=str)
    args = parser.parse_args()
    plot_data(args.path)
