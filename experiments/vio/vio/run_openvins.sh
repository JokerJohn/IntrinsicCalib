#!/usr/bin/env bash
# OpenVINS on one bag:
#   run_openvins.sh <bag> <set_dir>/openvins/estimator_config.yaml <out.tum> [mono|stereo=mono] [serial|realtime=serial] [rate=1.0]
# serial  : deterministic ros1_serial_msckf on the raw-image VIO bag ($RIGC_DIR/flir_eval_rawbags/<key>_vio.bag,
#           made by scripts/make_raw_bag.py; the original bag path is mapped automatically).
# realtime: rosbag play --clock + image republish (timing-dependent, kept for reference).
# Intrinsics come from kalibr_imucam_chain.yaml next to the estimator config (make_vio_config.py).
# Outputs: <out>.tum (/ov_msckf/poseimu = IMU pose), <out>.log, <out>.pose.bag
set -uo pipefail
BAG=$(readlink -f "$1"); CFG=$(readlink -f "$2"); OUT=$(readlink -f -m "$3"); MODE=${4:-mono}; PLAY=${5:-serial}; RATE=${6:-1.0}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RAWDIR=${RIGC_DIR:-$HERE/../../../data/rigc}/flir_eval_rawbags   # derived raw-image / rectified bags (generated)
# VIO_WS: catkin workspace with OpenVINS and VINS-Fusion built (see README)
source /opt/ros/noetic/setup.bash; source "${VIO_WS:?set VIO_WS to the OpenVINS/VINS-Fusion catkin workspace}/devel/setup.bash"; source "$HERE/../scripts/ros_env.sh"
mkdir -p "$(dirname "$OUT")"; STEM=${OUT%.tum}; rm -f "$STEM.pose.bag" "$STEM.pose.bag.active"
NCAM=1; STEREO=false; if [ "$MODE" = stereo ]; then NCAM=2; STEREO=true; fi
DUR=$(python3 -c "import rosbag,sys; b=rosbag.Bag(sys.argv[1]); print(int(b.get_end_time()-b.get_start_time()+0.999))" "$BAG")
export ROS_LOG_DIR="$STEM.roslog"; mkdir -p "$ROS_LOG_DIR"
if [ "$PLAY" = serial ]; then
  case "$BAG" in *flir_eval_rawbags*) RAW="$BAG";;   # a raw/rectified VIO bag given directly
    *) KEY=$(basename "$BAG" .bag); RAW="$RAWDIR/${KEY}_vio.bag";; esac
  if [ ! -s "$RAW" ]; then echo "[run_openvins] making raw-image bag $RAW"; python3 "$HERE/../scripts/make_raw_bag.py" "$BAG" "$RAW" || exit 1; fi
  TMO=$((DUR*3+60))
  echo "[run_openvins] SERIAL bag=$RAW ($DUR s) config=$CFG mode=$MODE timeout=${TMO}s master port=$ROS_PORT"
  timeout -s INT --kill-after=20 "$TMO" roslaunch -p "$ROS_PORT" "$HERE/openvins_serial.launch" \
    config_path:="$CFG" bag:="$RAW" out_bag:="$STEM.pose.bag" max_cameras:="$NCAM" use_stereo:="$STEREO" > "$STEM.log" 2>&1
else
  TMO=$(python3 -c "print(int($DUR/$RATE)+40)")
  echo "[run_openvins] REALTIME bag=$BAG ($DUR s) config=$CFG mode=$MODE rate=$RATE timeout=${TMO}s master port=$ROS_PORT"
  timeout -s INT --kill-after=20 "$TMO" roslaunch -p "$ROS_PORT" "$HERE/openvins.launch" \
    config_path:="$CFG" bag:="$BAG" out_bag:="$STEM.pose.bag" rate:="$RATE" max_cameras:="$NCAM" use_stereo:="$STEREO" > "$STEM.log" 2>&1
fi
echo "[run_openvins] roslaunch exited rc=$? (124 = timeout)"; sleep 1
python3 "$HERE/../scripts/bag_to_tum.py" "$STEM.pose.bag" "$OUT" /ov_msckf/poseimu || exit 1
python3 "$HERE/../scripts/traj_summary.py" "$OUT"
