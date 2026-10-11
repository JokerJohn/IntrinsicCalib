# Rig-C ground truth: `gt_rigc_s1.txt`, `gt_rigc_s2.txt`, `gt_rigc_s3.txt`

These trajectories are the reference of the Rig-C columns of Table III. They are
LiDAR-inertial odometry (FAST-LIO2) refined with manual loop closures.

## Files

| file | sequence (`$RIGC_DIR`) | rows | span / path |
|---|---|---|---|
| `gt_rigc_s1.txt` | `2026-02-23-11-46-32.bag` | 258 | 25.7 s / 17.3 m (open path, no revisit) |
| `gt_rigc_s2.txt` | `2026-02-23-11-57-46.bag` | 958 | 95.7 s / 99.3 m (figure eight, return to the start) |
| `gt_rigc_s3.txt` | `2026-02-22-11-25-57.bag` | 708 | 70.7 s / 82.4 m (return to the start) |

* Format: TUM, `t x y z qx qy qz qw`, one row per Ouster scan that FAST-LIO2 processed (the IMU stream ends 1.0-1.3 s
  before the LiDAR stream, so the last 11-15 scans have no pose).
* Stamps: FAST-LIO2 `lidar_end_time` (time of the last point kept in array order; 25-60 ms before the end of the full scan,
  because the lowest ring lies largely inside the 4 m blind radius). `evaluate.py` resamples the reference at the VIO
  stamps in either case.
* World frame: `camera_init` = the IMU frame at the first scan. Body frame: the main IMU `/sensors/imu0`, with
  T_imu_lidar = inv(T_cam0_imu, Kalibr) . T_cam0_lidar (FAST-Calib), i.e. `extrinsics/derived_extrinsics.yaml`; this is the
  frame of the earlier reference `ref/*_lio.tum` and the IMU frame of the OpenVINS evaluation.

## Construction

1. FAST-LIO2 (`rigc_ouster64` configuration) on the Ouster/IMU stream; keyframes every 0.5 m / 10 deg.
2. Manual loop closures in a pose-graph GUI (iSAM2 backend; odometry edges with sigma_t 1 cm, sigma_R 1 mrad; loop edges with
   sigma 2 cm / 0.2 deg, the rounded RMS difference, 2.05 cm / 0.16 deg, between 21 manually measured loops of five
   sessions and an independent GICP re-measurement). Each loop is a GICP registration of two keyframe scans.
3. The keyframe correction is interpolated in time and applied to every raw FAST-LIO2 pose (one row per scan).

| file | loops (keyframe indices) |
|---|---|
| `gt_rigc_s1.txt` | 38->1 (an 11 m registration along an open path, not a revisit; it agrees with the odometry within 1.9 cm) |
| `gt_rigc_s2.txt` | figure-eight loops 67->30, 68->26, 69->24, 71->23, and the return-to-start loop 230->0 (target = keyframes 0-5); the edge 179->166 is not used: the two keyframes are 3 s apart on the same pass, so it is a second odometry measurement and not a loop |
| `gt_rigc_s3.txt` | seven return-to-start loops 151->12, 155->7, 157->4, 158->3, 161->1, 165->0, 174->0 |

## Accuracy

Start->end: inv(T_first) T_last against the GICP of the last to the first keyframe scan (S2: 0.912 m / 6.32 deg,
S3: 0.817 m / 4.81 deg). In S3 and in the S2 reference this pair is itself constrained by a loop, so the column measures
consistency rather than independent accuracy. Cross-time residual: planar points of each keyframe scan against the nearest
point of scans at least 10 s apart (point-to-plane), all points / within 5 m of the loop places.

| sequence | trajectory | start->end error [cm / deg] | cross-time all, median / p95 [cm] | cross-time at the loops, median / p95 [cm] | ATE to raw FAST-LIO2 [cm] |
|---|---|---|---|---|---|
| S1 | **reference** | - | 1.0 / 4.2 | 0.8 / 5.1 | 0.2 |
| S1 | raw FAST-LIO2 | - | 1.1 / 4.3 | 0.9 / 5.2 | 0 |
| S2 | **reference** | 2.3 / 0.09 | 1.4 / 6.9 | 1.6 / 8.1 | 1.3 |
| S2 | raw FAST-LIO2 | 8.1 / 0.22 | 1.5 / 7.1 | 1.8 / 8.2 | 0 |
| S3 | **reference** | 1.6 / 0.17 | 1.2 / 7.3 | 1.0 / 5.5 | 1.1 |
| S3 | raw FAST-LIO2 | 5.6 / 0.21 | 1.5 / 7.3 | 1.5 / 5.6 | 0 |

FAST-LIO2 drifts by only a few centimetres on these runs, so the loop closure changes the trajectories by 0.2-1.3 cm (ATE).
SE(3) ATE of these references against the earlier FAST-LIVO2 references `../ref/*_lio.tum`: S1 0.7 cm, S2 3.1 cm
(max 10.2 cm), S3 2.7 cm.
