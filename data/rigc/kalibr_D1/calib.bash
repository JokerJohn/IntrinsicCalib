#!/bin/bash
# Kalibr commands used for this session (run inside a Kalibr ROS workspace).
rosrun kalibr kalibr_calibrate_cameras --target april.yaml --models pinhole-radtan pinhole-radtan --topics /sensors/cam0/compressed /sensors/cam1/compressed --bag calib.bag
rosrun kalibr kalibr_calibrate_imu_camera --target april.yaml --imu imu.yaml --cam calib-camchain.yaml --bag calib.bag
