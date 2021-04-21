import numpy as np

NoiseGyro = 1.7e-4 #1.6968e-04 
NoiseAcc = 2.0000e-3 #2.0e-3
GyroWalk = 1.9393e-05 
AccWalk = 3.0000e-03 # 3e-03
Frequency = 200 # True for collected data
servoNoise = 1 / 180 * np.pi