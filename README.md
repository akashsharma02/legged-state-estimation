# Legged Robot State Estimation through combined forward kinematic and pre-integrated contact factors

This repository is a reimplementation of the paper [Legged Robot State estimation through combined forward kinematic and Preintegrated Contact Factors](https://arxiv.org/abs/1712.05873)

## Install Requirements

| Package                   | Installation mechanism  | Command                                                    |
| -----------               | -----------             | ----                                                       |
| GCC 7+ (C++17 compatible) | apt-get (Ubuntu/Debian) | `sudo apt-get install gcc` / `sudo apt-get install gcc-9`  |
| CMake 3.15+               | Make                    | [CMake Install](https://cmake.org/install/)                |
| Conan                     | Pip/conda               | `pip install conan` / `conda install -c conda-forge conan` |
| Eigen3                    | CMake                   | [Eigen Install](#TODOLink)                                 |
| GTSAM 4.1.0               | CMake                   | [GTSAM Install](#TODOLink)                                 |

Remaining used packages should get built automatically during cmake configuration. Prefer `ccmake` for viewing all available options

## Usage

# The currently working executable is `inertialOnly`, coupled with `sequence10.txt`

```
./build/bin/inertial -c dataset/config/spot_mini_mini.yaml -d dataset/data/raw_collected/sequence1.txt -i dataset/config/imu_fake.yaml -b false -o evaluation/inertialOnlyTrajectory.txt -m 24000

./bin/inertialOnly -c ../dataset/config/spot_mini_mini.yaml -d ../dataset/data/raw_collected/sequence10.txt  -i ../dataset/config/imu_fake.yaml -m 24000 -b false -o ../evaluation/Trajectory.txt
```
Writes to `evaluation/inertialOnlyTrajectory.txt`, then go into `evaluation`

```
python plot_traj.py
```
to view trajectory.

## Developers

For logging use the CMake option `LOGGER_ACTIVE_LEVEL` to set the logging level during compile time
Example:
```
DEBUG("Verifying first factor key for robot {}: {}", i, keys);
```


