#!/usr/bin/env bash
# VINS-Fusion headless on one bag:
#   run_vins.sh <bag> <set_dir>/vins/vins_mono_imu.yaml|vins_stereo_imu.yaml <out.tum> [rate=1.0]
# Intrinsics come from cam0.yaml/cam1.yaml next to the config (make_vio_config.py).
# Outputs: <out>.tum (/vins_estimator/odometry = IMU/body pose in VINS world), <out>.log, <out>.pose.bag
set -uo pipefail
BAG=$(readlink -f "$1"); CFG=$(readlink -f "$2"); OUT=$(readlink -f -m "$3"); RATE=${4:-1.0}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/noetic/setup.bash; source "${VIO_WS:?set VIO_WS to the OpenVINS/VINS-Fusion catkin workspace}/devel/setup.bash"; source "$HERE/../scripts/ros_env.sh"
mkdir -p "$(dirname "$OUT")" /tmp/vins_output; STEM=${OUT%.tum}; rm -f "$STEM.pose.bag" "$STEM.pose.bag.active"
DUR=$(python3 -c "import rosbag,sys; b=rosbag.Bag(sys.argv[1]); print(int(b.get_end_time()-b.get_start_time()+0.999))" "$BAG")
TMO=$(python3 -c "print(int($DUR/$RATE)+40)")
REPUB=true; case "$BAG" in *flir_eval_rawbags*) REPUB=false;; esac
export ROS_LOG_DIR="$STEM.roslog"; mkdir -p "$ROS_LOG_DIR"
echo "[run_vins] bag=$BAG ($DUR s) config=$CFG rate=$RATE timeout=${TMO}s master port=$ROS_PORT"
timeout -s INT --kill-after=20 "$TMO" roslaunch -p "$ROS_PORT" "$HERE/vins.launch" \
  config_path:="$CFG" bag:="$BAG" out_bag:="$STEM.pose.bag" rate:="$RATE" republish:="$REPUB" > "$STEM.log" 2>&1
echo "[run_vins] roslaunch exited rc=$? (124 = timeout)"; sleep 1
python3 "$HERE/../scripts/bag_to_tum.py" "$STEM.pose.bag" "$OUT" /vins_estimator/odometry || exit 1
python3 "$HERE/../scripts/traj_summary.py" "$OUT"
