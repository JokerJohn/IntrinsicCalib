# Kalibr baseline on the public sessions

This baseline is not reported in the paper; Kalibr appears only in Tables II and III (Rig-C and Rig-D).

Kalibr (`kalibr_calibrate_cameras`, pinhole-radtan, the M2 model) is run at its
shipped defaults on exactly the training images of every fold of the rotating
4-fold holdout of `final_comparison.py` (evaluation fold = `entries[f::4]` of
the images sorted by name; the rest is the training set), and its camera chain
is scored with `protocol.pose_only` on the legacy-extractor corners of the
evaluation fold. The same scoring is applied to every other method of the
table, so the cells are comparable. For Rig-A (`rig_a/train33`) the
training set is the 33 images outside the six validation images
(`left-0000/0006/0011/0019/0024/0035`).

## Files

| file | role |
|---|---|
| `Dockerfile_ros1_20_04.retry` | the image recipe that was built (see below) |
| `make_bag.py` | runs inside the container: images -> ROS1 bag, one `sensor_msgs/Image` (mono8) per image on `/cam0/image_raw`, stamps 0.1 s apart |
| `run_kalibr.py` | `run`: one container per (session, fold); `score`: parses the Kalibr log and camchain and scores the fold; writes `<runs>_folds_raw.json` |
| `make_table.py` | assembles `kalibr_table1.json` from the primary and the repeat run |
| `sanity_check.py` | checks that the fold construction and the scoring reproduce the `ours-M2` row, and scores the ROS and mrcal fits on the six Rig-A validation images |
| `kalibr_table1.json` | the numbers of the table: per-fold held-out errors, mean and std (ddof = 0) over the folds, views used by Kalibr, the camera matrix and distortion of every fold |

## Image

Kalibr runs from a docker image built from the `Dockerfile_ros1_20_04` of
Kalibr's own repository (`https://github.com/ethz-asl/kalibr`, branch `master`,
commit `94bb8437a72a0d97a491097a7085bf3db4f93bba`), tagged `kalibr:noetic`.
`Dockerfile_ros1_20_04.retry` is that file with one added `RUN` line that sets
`Acquire::Retries "20"` and, only when the build argument `UBUNTU_MIRROR` is
given, replaces the Ubuntu apt mirror (the download is then more robust; the
package list, the catkin workspace and the build, `catkin build` in Release,
are unchanged). Build from the
root of the Kalibr checkout, which is the build context (`ADD . $WORKSPACE/src/kalibr`):

```bash
git clone https://github.com/ethz-asl/kalibr.git && cd kalibr
cp <this directory>/Dockerfile_ros1_20_04.retry .
docker build -f Dockerfile_ros1_20_04.retry -t kalibr:noetic .   # optional: --build-arg UBUNTU_MIRROR=<url>
```

(The unmodified `Dockerfile_ros1_20_04` builds the same image.) The image name can be changed with `KALIBR_IMAGE`.

## Command

For each job `run_kalibr.py` writes `checkerboard.yaml`

```yaml
target_type: 'checkerboard'
targetCols: <inner corners, columns>
targetRows: <inner corners, rows>
rowSpacingMeters: <square size [m]>
colSpacingMeters: <square size [m]>
```

and runs, in a container started with `--network none` and the entrypoint
replaced by `/bin/bash` (so that `KALIBR_MANUAL_FOCAL_LENGTH_INIT` of the
upstream entrypoint is not set):

```bash
source /catkin_ws/devel/setup.bash
python3 make_bag.py train.bag <training images>
rosrun kalibr kalibr_calibrate_cameras --models pinhole-radtan \
    --target checkerboard.yaml --bag train.bag --topics /cam0/image_raw --dont-show-report
```

No other option is set: Kalibr's corner extraction, its outlier-corner removal
and its view selection are used as shipped. The view selection draws views at
random without a seed, so two executions on the same images differ slightly;
the table is therefore produced twice (`runs/` and, with
`KALIBR_RUNS=runs_repeat2`, `runs_repeat2/`), and both are stored in
`kalibr_table1.json` (`primary`, `repeat_run`). The cells of the two runs agree
to within 0.001 px on the five sessions (0.004 px on Rig-A).

## Running

```bash
cd experiments/public/experiment/kalibr_baseline
export PUBLIC_DIR=...  RIG_A_DIR=...          # defaults: experiments/public, data/rig_a
python run_kalibr.py run  -j 6                 # 6 containers in parallel; 21 jobs (5 sessions x 4 folds + Rig-A)
python run_kalibr.py score                     # -> runs_folds_raw.json
KALIBR_RUNS=runs_repeat2 python run_kalibr.py run -j 6
KALIBR_RUNS=runs_repeat2 python run_kalibr.py score
python sanity_check.py                         # needs out/fs39_ros.json and out/fs39_mrcal.json of the top-level README
python make_table.py                           # -> kalibr_table1.json
```

`run_kalibr.py` mounts this directory, `PUBLIC_DIR` and `RIG_A_DIR` into the
container at their own paths. `OMP_NUM_THREADS=1` is recommended for the
scoring.
