# Data manifest: Rig-C and Rig-D

Rig-C is the handheld sensor platform of Section V: two FLIR cameras
(1440 x 1080, about 113 deg field of view, 10 Hz, published as
`sensor_msgs/CompressedImage`), an Ouster OS-128 LiDAR (262 144 points per
scan, 10 Hz) and two IMUs (imu0 at 500 Hz, imu1 at 100 Hz). The bags are too
large for this repository and are hosted separately (see the top-level README
for the download location). Place them, with the names below, in one directory
and export `RIGC_DIR=<that directory>`; every script of `experiments/rigc/` and
`experiments/vio/` resolves the bags through this variable (default:
`data/rigc/`).

## Bags

| file in `$RIGC_DIR` | role | duration | size [bytes] | md5 of the first 1 MiB | cam0 / cam1 / lidar0 messages |
|---|---|---|---|---|---|
| `calib_D1.bag` | calibration session D1 (AprilGrid, 2026-02-22) | 58.1 s | 4 107 628 260 | `2b009bf694135e3c3444642e0baa738d` | 570 / 572 / 582 |
| `calib_D2.bag` | calibration session D2 (AprilGrid, 2026-02-23) | 42.2 s | 2 898 588 212 | `56273e162cfc02db8fc15734c29f1443` | 416 / 416 / 423 |
| `2026-02-22-11-25-57.bag` | VIO sequence S3, 72 s, 82 m indoor path (evaluated with the D1 intrinsics) | 72.1 s | 5 157 725 630 | `6d48300e9acc38eadd265faa7ff1921f` | 716 / 717 / 722 |
| `2026-02-23-11-46-32.bag` | VIO sequence S1, 27.5 s, 17 m indoor path (D2 intrinsics) | 27.5 s | 1 954 574 469 | `ec9edf4de3ef7848cbe48d172889006a` | 265 / 266 / 276 |
| `2026-02-23-11-57-46.bag` | VIO sequence S2, 97 s, 99 m indoor path (D2 intrinsics) | 97.2 s | 6 697 070 990 | `e8531135131d36dbcd215dce2e727964` | 962 / 962 / 973 |

`md5 of the first 1 MiB` is `head -c 1048576 <bag> | md5sum`. The five Rig-C
bags are rosbag 2.0, lz4-compressed, with the same topics:

| topic | type | rate |
|---|---|---|
| `/sensors/cam0/compressed`, `/sensors/cam1/compressed` | `sensor_msgs/CompressedImage` | 10 Hz |
| `/sensors/imu0` | `sensor_msgs/Imu` | 500 Hz |
| `/sensors/imu1` | `sensor_msgs/Imu` | 100 Hz |
| `/sensors/lidar0` | `sensor_msgs/PointCloud2` (x, y, z float32; intensity uint8; time float64 in ns since scan start; ring uint16) | 10 Hz |
| `/sensors/mag0`, `/sensors/mag1` | `sensor_msgs/MagneticField` | 500 / 100 Hz |

## Rig-D sequence (Table III, monocular column)

The Rig-D platform is a wheeled robot with a camera of the Rig-A model, a
solid-state LiDAR and an IMU. One indoor sequence (163 s, 126.4 m, closed
loop) is used, only for the visual-inertial odometry experiment
(`experiments/vio/rigd/`). The bag is hosted with the Rig-C bags; place it in
`$RIGC_DIR`.

| file in `$RIGC_DIR` | role | duration | size [bytes] | md5 of the first 1 MiB | image / lidar / imu messages |
|---|---|---|---|---|---|
| `rigd_office1.bag` | Rig-D VIO sequence (indoor office loop, robot static for the first 9 s and the last 12 s) | 163.4 s | 11 347 933 274 | `b01817dbc6fd9fdf6c10a2f3a00b0d8c` | 1634 / 1633 / 32253 |

rosbag 2.0, not compressed:

| topic | type | rate |
|---|---|---|
| `/nv12_frame` | `sensor_msgs/Image` (rgb8, 1920 x 1080) | 10 Hz |
| `/echo1_roi0` | `sensor_msgs/PointCloud2` (x, y, z, intensity float32; timestamp float64 in microseconds; 22 409 points per scan, point step 32) | 10 Hz |
| `/imu_raw` | `sensor_msgs/Imu` (accelerometer in g) | about 197 Hz |

`experiments/vio/rigd/scripts/split_bag.py` converts the bag once into the two
inputs of the experiment (LiDAR + IMU; mono8 images + IMU for the VIO runs).
The ground truth (`experiments/vio/gt/gt_rigd_office1.txt`) and the camera-LiDAR
extrinsic (`experiments/vio/rigd/extrinsics/extrinsic_cam_lidar.yaml`) ship with
the repository.

## Files that ship with the repository

* `kalibr_D1/`, `kalibr_D2/`: the Kalibr input (`april.yaml`, `imu*.yaml`,
  `calib.bash`) and output (`calib-camchain*.yaml`, `calib-results*.txt`,
  `calib-imu*.yaml`) of each calibration session. The published Kalibr
  intrinsics used in Table II and the camera-IMU extrinsics and time offsets
  used by the VIO configurations come from these files. The AprilGrid is
  3 x 3 tags of 0.115 m with a spacing ratio of 0.3 (`april.yaml`).
* `../../experiments/rigc/corners_D{1,2}_cam{0,1}.json`: the corner
  observations extracted from the two calibration bags (raw Kalibr corners and
  the refined corners of the proposed extractor for every frame). With these
  caches `experiments/rigc/flir_full.py` reproduces Table II without the bags.
* `frames/D2_cam0_0337.png`: frame 337 of session D2, cam0 (used by
  `tests/test_aprilgrid.py`).
* `../../experiments/rigc/intrinsics/`: the intrinsics of the sets Kalibr,
  ours-M2 and ours (Full) per camera and session, written by `flir_full.py`;
  input of the VIO experiment.
* `../../experiments/vio/gt/`: the ground-truth trajectories of Table III
  (Rig-C: FAST-LIO2 with manual loop closures; Rig-D: PALoc prior-map
  localisation with manual loop closures).
* `../../experiments/vio/results/trajectories/`: the OpenVINS trajectories of
  Table III.

## Derived files that the scripts generate

* `flir_eval_rawbags/<key>_vio.bag`: image + IMU bags without LiDAR
  (`experiments/vio/scripts/make_raw_bag.py`).
* `flir_eval_rawbags/<set>/<key>_vio_rect.bag`: the same bags rectified with
  one intrinsics set (`experiments/vio/scripts/make_rect_bag.py`, called by
  `run_matrix.py`).
* `experiments/rigc/frames_<session>_<cam>.npz`: decoded frame caches of the
  calibration bags (written by `flir_full.py` on first use, 0.3 to 0.7 GB
  each).
