#!/usr/bin/env bash
# Run FAST-LIVO2 headless on one bag and write a TUM trajectory.
#   run_one.sh <bag> <config.yaml> <camera.yaml> <out.tum> [rate=1.0]
# Outputs next to <out.tum>:
#   <out>.tum          FAST-LIVO2 native pose file (evo/pose_output_en, state after every LIO/VIO update)
#   <out>.odom.tum     /aft_mapped_to_init converted from the recorded bag (fallback if native file is missing)
#   <out>.log          console log of the roslaunch (grep "VIO"/"exposure"/"Get image" to confirm images are used)
#   <out>.odom.bag     recorded odometry topic
set -uo pipefail
BAG=$(readlink -f "$1"); CFG=$(readlink -f "$2"); CAM=$(readlink -f "$3"); OUT=$(readlink -f -m "$4"); RATE=${5:-1.0}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# FASTLIVO_WS: catkin workspace that contains src/FAST-LIVO2 (patched for the Ouster point layout, see README)
FASTLIVO_WS=${FASTLIVO_WS:?set FASTLIVO_WS to the FAST-LIVO2 catkin workspace}
FASTLIVO_SRC=$FASTLIVO_WS/src/FAST-LIVO2
source /opt/ros/noetic/setup.bash
source "$FASTLIVO_WS/devel/setup.bash"
source "$HERE/scripts/ros_env.sh"   # private rosmaster on $ROS_PORT
mkdir -p "$(dirname "$OUT")" "$FASTLIVO_SRC/Log/result"
STEM=${OUT%.tum}
SEQ=flir_eval_$(basename "$STEM")_$$
NATIVE="$FASTLIVO_SRC/Log/result/$SEQ.txt"
rm -f "$NATIVE" "$STEM.odom.bag" "$STEM.odom.bag.active"

DUR=$(python3 -c "import rosbag,sys; b=rosbag.Bag(sys.argv[1]); print(int(b.get_end_time()-b.get_start_time()+0.999))" "$BAG")
TMO=$(python3 -c "print(int($DUR/$RATE)+40)")
export ROS_LOG_DIR="$STEM.roslog"; mkdir -p "$ROS_LOG_DIR"
echo "[run_one] bag=$BAG ($DUR s) config=$CFG camera=$CAM rate=$RATE timeout=${TMO}s seq=$SEQ master port=$ROS_PORT"
timeout -s INT --kill-after=20 "$TMO" roslaunch -p "$ROS_PORT" "$HERE/flir.launch" \
  config:="$CFG" camera_yaml:="$CAM" bag:="$BAG" out_bag:="$STEM.odom.bag" seq_name:="$SEQ" rate:="$RATE" \
  > "$STEM.log" 2>&1
RC=$?
echo "[run_one] roslaunch exited rc=$RC (124 = timeout)"
# make sure nothing lingers
sleep 1

if [ -f "$STEM.odom.bag" ]; then
  python3 "$HERE/scripts/bag_to_tum.py" "$STEM.odom.bag" "$STEM.odom.tum" /aft_mapped_to_init
fi
if [ -s "$NATIVE" ]; then
  cp "$NATIVE" "$OUT"; echo "[run_one] native pose file -> $OUT ($(wc -l < "$OUT") poses)"
elif [ -s "$STEM.odom.tum" ]; then
  cp "$STEM.odom.tum" "$OUT"; echo "[run_one] WARNING: native pose file missing, using odometry topic -> $OUT"
else
  echo "[run_one] ERROR: no trajectory produced"; exit 1
fi
python3 "$HERE/scripts/traj_summary.py" "$OUT"
echo "[run_one] images received: $(grep -c 'Get image' "$STEM.log")  VIO log lines: $(grep -ci 'vio\|exposure' "$STEM.log")"
