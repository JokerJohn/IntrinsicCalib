# Rig-D ground truth: `gt_rigd_office1.txt`

This trajectory is the reference of the Rig-D column of Table III. It is LiDAR localisation in a prior terrestrial-laser-scan
(TLS) map, refined with manual loop closures.

## File

| item | value |
|---|---|
| sequence | `rigd_office1.bag` (Rig-D, indoor office loop, 162.9 s, 126.4 m; static before t0+9.4 s and after t0+151.5 s; the robot ends at its start pose) |
| format | TUM, `t x y z qx qy qz qw`, one row per LiDAR scan (`/echo1_roi0`), 1630 rows, t0 = 1785481431.621393; the stamps are the scan stamps of FAST-LIO2 and of the earlier FAST-LIVO2 reference (identical within 1 us) |
| world frame | frame of the TLS map of the office (125.8 M points; the localisation uses a 0.2 m voxel copy) |
| body frame | Rig-D LiDAR frame, which is the IMU frame of the Rig-D VIO evaluation: the OpenVINS configurations use T_cam_imu = T_cam_lidar . T_lidar_imu with T_lidar_imu = I, as does the earlier reference `../ref/rigd_office1_lio.tum` (extrinsic_T = 0) |

The estimator runs in the FAST-LIO2 IMU frame (extrinsic_R = I, extrinsic_T = t = [0.00569, -0.03299, 0.03694] m, the value of
another unit of the same model; p_imu = p_lidar + t). The file therefore stores T_world_lidar = T_world_imu . [I | t]. The lever arm is not verified for
Rig-D; expressing one trajectory in either frame changes the SE(3)-aligned ATE by 3.3 cm.

## Construction

1. Localisation in the prior TLS map: front end = FAST-LIO2 (odometry bit-identical to the plain FAST-LIO2 run), map
   registration of every scan; automatic loop closure disabled because it accepted 23 false loops in a first run.
2. Pose graph: the odometry and zero-motion edges plus 1251 map priors at the scans whose map registration has fitness >= 0.7
   and agrees with the localisation within 0.3 m / 2 deg.
3. Five manual loops (GICP of keyframe scans): 1459->5, 1460->90, 1514->99, 1517->100, 1519->101 (end of the run against the
   static start). An independent GICP re-measurement agrees with them to 0.7-5.3 cm / 0.11-0.34 deg.
4. Re-optimisation with iSAM2, loop sigma 2 cm / 0.2 deg (the RMS difference between 21 manually measured loops of five
   sessions and an independent GICP re-measurement is 2.05 cm / 0.16 deg). The first scan (the initialisation scan) is added
   with the FAST-LIO2 relative motion to the second scan.

## Accuracy and choice of the reference

Start->end truth: GICP of the last to the first keyframe scan (both static at the start pose), 0.046 m / 3.71 deg.
Map metric: the stitched map of the 251 FAST-LIO2 keyframe scans (0.1 m voxel, the same point set for every trajectory)
is registered once to the TLS map (5 cm voxel crop): start pose anchored to the localisation's start pose, then rigid
point-to-plane ICP (0.5 m, 0.2 m). NN = point-to-point distance (mean / median / p95 over the 94.5 % of points with a map
point within 1 m), inlier = fraction of all points within 0.1 / 0.2 m, p2pl = point-to-plane distance with TLS normals.
Cross-time residual: planar points of each scan against the nearest point of scans at least 10 s apart (point-to-plane).

| trajectory | start->end error [cm / deg] | z range [m] | TLS NN mean / median / p95 [cm] | inlier 0.1 / 0.2 m | p2pl median [cm] | cross-time median / p95 [cm] |
|---|---|---|---|---|---|---|
| **gt_rigd_office1** (map localisation + manual loops) | 1.4 / 0.29 | 0.13 | 9.1 / 4.3 / 35.6 | 0.736 / 0.854 | 3.1 | 3.7 / 21.1 |
| map localisation without manual loops | 0.4 / 0.27 | 0.13 | 9.1 / 4.4 / 35.0 | 0.734 / 0.855 | 3.1 | 3.8 / 21.1 |
| FAST-LIO2 + 3 manual loops | 2.3 / 0.26 | 0.79 | 13.7 / 8.6 / 44.4 | 0.536 / 0.766 | 7.0 | 4.1 / 21.4 |
| FAST-LIO2 raw odometry | 275.7 / 6.21 | 2.10 | 19.7 / 11.4 / 69.9 | 0.428 / 0.613 | 8.6 | 5.2 / 26.7 |
| FAST-LIVO2 LIO (earlier reference) | 29.7 / 1.39 | 0.62 | 11.7 / 7.4 / 39.1 | 0.602 / 0.822 | 5.9 | 4.1 / 36.8 |

The localisation's TLS distance is not independent of it, because its map priors come from a 0.2 m copy of the same map; the
map-independent columns (start->end error, z range of a flat-floor run, cross-time residual) rank the trajectories in the
same order. The manual loops change the localisation result by 0.6 cm (ATE). A single start/end loop cannot make FAST-LIO2
equivalent: the vertical drift (z range 0.79 m) and the distribution of the 6 deg yaw drift along the path are not observable
from one revisit. SE(3) ATE of this reference against the earlier FAST-LIVO2 reference `../ref/rigd_office1_lio.tum`: 16.1 cm
RMSE (max 29.8 cm), 1.48 deg.
