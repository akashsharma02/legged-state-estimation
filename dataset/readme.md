## Decompress Data

Decompress data folder so that it looks like
```
dataset/
    data
        raw_measurement
        ground_truth
        ...
    config.py
    separate.py
    ...
```

## Verify Sanity of the data

`python raw_data_vis.py -p data/raw_collected`

To show approximate ground truth plots. There are some small vertical displacements over time,
I tried eliminating them through more careful control but they don't seem to be going away.
I suspect it is something with the Bezier control, and since they are in the scale of 0.01m,
I feel like it is relatively trivial.

## Simulating IMU Noise

Source from [ETHZ-ASL](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)

**Simulator** by default runs on 100Hz, I doubled the frequency to 200Hz to get a more refined

## Gravity Measurement

The generated "IMU Measurement" doesn't have gravity component. Preintegrated IMU can handle tunable
gravity vector.
