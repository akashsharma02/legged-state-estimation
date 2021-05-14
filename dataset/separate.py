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

def separate_and_add_noise(args, path):
    full_path = os.path.join(args.path, path)
    df = pd.read_csv(full_path)
    # df[['ax','ay','az']] *= 200
    # gt = df[['ts', 'x', 'y', 'z', 'i', 'j', 'k', 'w']]
    # gt = gt[::10]
    # gt.to_csv(os.path.join(args.ground_truth, path), index=False)

    # {f,b} (front, back) {l,r} (left, right), 0,1,2,contact
    # IMU\
    # Front Leg Encoder\
    # Back Leg encoder\
    # Contact Sensors
    measurement_df = df[["ts","x", "y","z","i","j","k","w",\
        "wx",'wy','wz','ax','ay','az',\
        'fl0','fl1','fl2','fr0','fr1','fr2',\
        'bl0','bl1','bl2','br0','br1','br2',\
        'flc','frc','blc','brc']]
    
    b_w = np.array([-0.0,0.0,0.0])
    b_a = np.array([-0.0,0.0,0.0])

    dt = 1. / config.Frequency
    gyro_stdev = config.NoiseGyro / np.sqrt(dt)
    acc_stdev = config.NoiseAcc / np.sqrt(dt)

    gyro_additive_noise = np.random.normal(0, gyro_stdev, (len(df), 3))
    acc_additive_noise = np.random.normal(0, acc_stdev, (len(df), 3))
    encoder_additive_noise = np.random.normal(0, config.servoNoise, (len(df), 12))

    translation_additive_noise = np.random.normal(0, config.translationNoise, (len(df), 3))
    rotation_perturbation = np.random.normal(0, config.quaternionPert, ((len(df), 4)))

    # Add additive noise
    measurement_df.iloc[:, 8:11] = measurement_df.iloc[:, 8:11] + gyro_additive_noise
    measurement_df.iloc[:, 11:14] = measurement_df.iloc[:, 11:14] + acc_additive_noise
    measurement_df.iloc[:, 14:26] = measurement_df.iloc[:, 14:26] + encoder_additive_noise
    measurement_df.iloc[:, 1:4] = measurement_df.iloc[:, 1:4] + translation_additive_noise
    measurement_df.iloc[:, 4:8] = measurement_df.iloc[:, 4:8] + rotation_perturbation
    # print(measurement_df.iloc[:, 4:8] / np.linalg.norm(measurement_df.iloc[:, 4:8].to_numpy(), axis=1)[:, np.newaxis])
    measurement_df.iloc[:, 4:8] = measurement_df.iloc[:, 4:8] / np.linalg.norm(measurement_df.iloc[:, 4:8].to_numpy(), axis=1)[:, np.newaxis]
        # No additive noise for contact

    measurement_df.iloc[0, 8:11] += b_w
    measurement_df.iloc[0, 11:14] += b_a
    for i in range(1, len(measurement_df)):
        gyro_bias = getBiasVector(config.GyroWalk) # (3, 1)
        b_w += gyro_bias
        measurement_df.iloc[i, 8:11] += gyro_bias
        acc_bias = getBiasVector(config.AccWalk)
        b_a += acc_bias
        measurement_df.iloc[i, 11:14] += + acc_bias
    
    measurement_df.to_csv(os.path.join(args.measurement, path), index=False)

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
    
    args = parser.parse_args()

    for fname in os.listdir(args.path):
        # full_path = os.path.join(args.path, fname)
        separate_and_add_noise(args, fname)
