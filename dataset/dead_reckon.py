import pandas as pd
import numpy as np
import os
import config # Contains Params
import argparse
import random
import matplotlib.pyplot as plt

def separate_and_add_noise(args):
    df = pd.read_csv(args.path)
    measurement_df = df[["ts","wx",'wy','wz','ax','ay','az']]


    length = 24000
    p = np.zeros((3, length))
    v = np.zeros((3, ))
    v_log = np.zeros((3, length))
    dt = 1. / 200
    print(f"{length} Measurements")
    for i in range(1, length):
        a = measurement_df.iloc[i, 4:].to_numpy()
        # Because simulator didn't div it over time
        a *= 200
        p[:, i] = p[:, i - 1] + v * dt + 0.5 * a * dt * dt
        v = v + a * dt

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter3D(p[0, :], p[1, :], p[2, :])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_xlim3d(-0.5, 3.5)
    ax.set_ylim3d(-4, 0.5)
    plt.show()

    np.savetxt("dr.csv", p.T, delimiter=",")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Awesome Visual SLAM")
    parser.add_argument("-p",
                        "--path",
                        default="data/perfect_measurement/sequence2.txt",
                        help="Path to the perfect measurements data",
                        type=str)
    
    args = parser.parse_args()
    separate_and_add_noise(args)
