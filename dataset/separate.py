from numpy.core.fromnumeric import size
import pandas as pd
import numpy as np
import os
import config # Contains Params
import argparse
import random

random.seed(0)
# https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model

def getBiasVector(stdDev):
    wt = np.random.normal(0, 1, size=(3,))
    # wt /= np.linalg.norm(wt)
    return stdDev * np.sqrt(1. / config.Frequency) * wt

def quat2Rod(quats):
    theta_2 = np.arccos(quats[:, 3])
    e = quats[:, :3] / np.sin(theta_2)[:, np.newaxis]
    rod = e * np.tan(theta_2)[:, np.newaxis]
    return rod

def rod2Quat(rod):
    rodN = np.linalg.norm(rod, axis=1)
    rode = rod / rodN[:, np.newaxis]
    theta_2 = np.arctan(rodN)
    numQuat = np.zeros((len(rodN), 4))
    sinT2 = np.sin(theta_2)[:, np.newaxis] * rode
    numQuat[:, :3] = sinT2
    numQuat[:, 3] = np.cos(theta_2)
    return numQuat

def separate_and_add_noise(args, path):
    full_path = os.path.join(args.path, path)
    df = pd.read_csv(full_path)
    df[['ax','ay','az']] *= 200
    gt = df[['ts', 'x', 'y', 'z', 'i', 'j', 'k', 'w']]
    gt.to_csv(os.path.join(args.ground_truth, path))

    # {f,b} (front, back) {l,r} (left, right), 0,1,2,contact
    # IMU
    # Front Leg Encoder
    # Back Leg encoder
    # Contact Frame Position Sensors
    # Contact Sensors
    measurement_df = df[["ts","wx",'wy','wz','ax','ay','az',\
        'fl0','fl1','fl2','fr0','fr1','fr2',\
        'bl0','bl1','bl2','br0','br1','br2',\
        'fli','flj','flk','flw',\
        'fri','frj','frk','frw',\
        'bli','blj','blk','blw',\
        'bri','brj','brk','brw',\
        'flc','frc','blc','brc']]
    measurement_df.to_csv(os.path.join(args.perfect_measurement, path))

    b_w = np.array([-0.003172,0.021267,0.078502])
    b_a = np.array([-0.025266,0.136696,0.075593])

    dt = 1. / config.Frequency
    gyro_stdev = config.NoiseGyro / np.sqrt(dt)
    acc_stdev = config.NoiseAcc / np.sqrt(dt)

    gyro_additive_noise = np.random.normal(0, gyro_stdev, (len(df), 3))
    acc_additive_noise = np.random.normal(0, acc_stdev, (len(df), 3))
    encoder_additive_noise = np.random.normal(0, config.servoNoise, (len(df), 12))

    # Add additive noise
    measurement_df.iloc[:, 1:4] = measurement_df.iloc[:, 1:4] + gyro_additive_noise
    measurement_df.iloc[:, 4:7] = measurement_df.iloc[:, 4:7] + acc_additive_noise
    measurement_df.iloc[:, 7:19] = measurement_df.iloc[:, 7:19] + encoder_additive_noise
    
    # measurement_df.iloc[0, 1:4] += b_w
    # measurement_df.iloc[0, 4:7] += b_a
    dbgdt = np.random.normal(0, 1, size=(len(measurement_df), 3))
    dbgdt = config.GyroWalk * np.sqrt(1. / config.Frequency) * dbgdt
    # Initial Condition
    dbgdt[0, :] += b_w
    gyro_bias = np.cumsum(dbgdt, axis=0)
    
    dbadt = np.random.normal(0, 1, size=(len(measurement_df), 3))
    dbadt = config.AccWalk * np.sqrt(1. / config.Frequency) * dbadt
    # Initial Condition
    dbadt[0, :] += b_a
    acc_bias = np.cumsum(dbadt, axis=0)

    measurement_df.iloc[:, 1:4] += gyro_bias
    measurement_df.iloc[:, 4:7] += acc_bias

    # Add Gyro Bias to contact frame estimation

    # for i in range(1, len(measurement_df)):
    #     gyro_bias = getBiasVector(config.GyroWalk) # (3, 1)
    #     b_w += gyro_bias
    #     measurement_df.iloc[i, 1:4] += gyro_bias
    #     acc_bias = getBiasVector(config.AccWalk)
    #     b_a += acc_bias
    #     measurement_df.iloc[i, 4:7] += + acc_bias

    rodNoise = np.random.normal(0, gyro_stdev, (len(df), 12))
    flquat = measurement_df.iloc[:, 19:23].to_numpy()
    flRod = quat2Rod(flquat) + rodNoise[:, :3]
    measurement_df.iloc[:, 19:23] += rod2Quat(flRod)
    frquat = measurement_df.iloc[:, 23:27].to_numpy()
    frRod = quat2Rod(frquat) + rodNoise[:, 3:6]
    measurement_df.iloc[:, 23:27] += rod2Quat(frRod)
    blquat = measurement_df.iloc[:, 27:31].to_numpy()
    blRod = quat2Rod(blquat) + rodNoise[:, 6:9]
    measurement_df.iloc[:, 27:31] += rod2Quat(blRod)
    brquat = measurement_df.iloc[:, 31:35].to_numpy()
    brRod = quat2Rod(brquat) + rodNoise[:, 9:12]
    measurement_df.iloc[:, 31:35] += rod2Quat(brRod)
    measurement_df.to_csv(os.path.join(args.measurement, path))

    return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Awesome Visual SLAM")
    parser.add_argument("-p",
                        "--path",
                        default=None,
                        help="Path to the data **folder**",
                        type=str)
    parser.add_argument("-g",
                        "--ground_truth",
                        default="data/ground_truth",
                        help="Path to the groundtruth folder",
                        type=str)
    parser.add_argument("-r",
                        "--measurement",
                        default="data/real_measurement",
                        help="Path to the measurement **folder**",
                        type=str)
    parser.add_argument("-u",
                        "--perfect_measurement",
                        default="data/perfect_measurement",
                        help="Path to the measurement **folder**",
                        type=str)
    
    args = parser.parse_args()

    for fname in os.listdir(args.path):
        separate_and_add_noise(args, fname)
