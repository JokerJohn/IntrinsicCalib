# Data

Large files are hosted separately; the small inputs ship with this repository. Download the data folder ([Baidu Netdisk](https://pan.baidu.com/s/1n3tDlzshlr6OiQmL6GJtyQ?pwd=vihg), extraction code `vihg`) and set `RIGC_DIR=<data>/bags` (Rig-A and Rig-B are also in `data/`).

| experiment | data | where | Download |
|---|---|---|---|
| Image and order selection, Sec. V-B, V-C | Rig-A, 39 images | `data/rig_a/` | — |
| Consistency check, Supplement B | Rig-B, 64 images | `data/rig_b/` | — |
| Public sessions, Table I | OpenCalib, ROS L/R, OpenCV L/R | `experiments/public/fetch_datasets.sh` (original sources) | — |
| Rig-C calibration and ablation, Table II | `calib_D1.bag` (4.1 GB), `calib_D2.bag` (2.9 GB) | external; corner caches `experiments/rigc/corners_*.json` reproduce Table II without the bags | [Baidu Netdisk](https://pan.baidu.com/s/1n3tDlzshlr6OiQmL6GJtyQ?pwd=vihg) (code `vihg`) |
| Rig-C VIO, Table III (S1, S2, S3) | `2026-02-23-11-46-32.bag` (2.0 GB), `2026-02-23-11-57-46.bag` (6.7 GB), `2026-02-22-11-25-57.bag` (5.2 GB) | external; reference trajectories `experiments/vio/gt/` | [Baidu Netdisk](https://pan.baidu.com/s/1n3tDlzshlr6OiQmL6GJtyQ?pwd=vihg) (code `vihg`) |
| Rig-D VIO, Table III | `rigd_office1.bag` (11.3 GB) | external; reference trajectory `experiments/vio/gt/`, extrinsic `experiments/vio/rigd/extrinsics/` | [Baidu Netdisk](https://pan.baidu.com/s/1n3tDlzshlr6OiQmL6GJtyQ?pwd=vihg) (code `vihg`) |

Sizes, checksums, durations and topics of the bags: [data/rigc/MANIFEST.md](data/rigc/MANIFEST.md). Faces in the Rig-A and Rig-B images are blurred outside the board; the calibration results are unchanged.
