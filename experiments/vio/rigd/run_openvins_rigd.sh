#!/usr/bin/env bash
# OpenVINS mono, deterministic serial reader (ros1_serial_msckf), on a Rig-D rectified bag.
#   run_openvins_rigd.sh <rect.bag> <cfg_dir>/openvins/estimator_config.yaml <out.tum>
# Same as ../vio/run_openvins.sh (serial branch) without the FLIR bag-path mapping. Private rosmaster, timeout.
# Outputs: <out>.tum (/ov_msckf/poseimu = IMU pose), <out>.log, <out>.pose.bag
set -uo pipefail
BAG=$(readlink -f "$1"); CFG=$(readlink -f "$2"); OUT=$(readlink -f -m "$3")
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/noetic/setup.bash; source "${VIO_WS:?set VIO_WS to the OpenVINS/VINS-Fusion catkin workspace}/devel/setup.bash"; source "$HERE/../scripts/ros_env.sh"
mkdir -p "$(dirname "$OUT")"; STEM=${OUT%.tum}; rm -f "$STEM.pose.bag" "$STEM.pose.bag.active"
DUR=$(python3 -c "import rosbag,sys; b=rosbag.Bag(sys.argv[1]); print(int(b.get_end_time()-b.get_start_time()+0.999))" "$BAG")
export ROS_LOG_DIR="$STEM.roslog"; mkdir -p "$ROS_LOG_DIR"
TMO=$((DUR*3+60))
echo "[run_openvins_rigd] SERIAL bag=$BAG ($DUR s) config=$CFG timeout=${TMO}s master port=$ROS_PORT"
timeout -s INT --kill-after=20 "$TMO" roslaunch -p "$ROS_PORT" "$HERE/../vio/openvins_serial.launch" \
  config_path:="$CFG" bag:="$BAG" out_bag:="$STEM.pose.bag" max_cameras:=1 use_stereo:=false > "$STEM.log" 2>&1
echo "[run_openvins_rigd] roslaunch exited rc=$? (124 = timeout)"; sleep 1
# kill anything left on this run's master port
pkill -f "__master:=http://localhost:$ROS_PORT" 2>/dev/null; pkill -f "roslaunch -p $ROS_PORT" 2>/dev/null; pkill -f "rosmaster.*-p $ROS_PORT" 2>/dev/null
python3 "$HERE/../scripts/bag_to_tum.py" "$STEM.pose.bag" "$OUT" /ov_msckf/poseimu || exit 1
python3 "$HERE/../scripts/traj_summary.py" "$OUT"
