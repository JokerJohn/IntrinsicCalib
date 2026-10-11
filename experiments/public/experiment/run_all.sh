#!/usr/bin/env bash
# Runs the scripts of experiments/public/experiment in sequence; the section
# "final table" produces Table I of the paper (see ../README.md).
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
mkdir -p out
SETS="../opencalib_front ../ros_wide_left ../ros_wide_right ../opencv_left ../opencv_right"

echo "########## ours (Full), full sessions (writes the shared corner cache) ##########"
for d in $SETS; do
  n=$(basename "$d")
  python3 rap_headless.py "$d" --out "out/${n}_rap.json" --corner-cache "out/${n}_cache.json"
done

echo
echo "########## ROS camera_calibration, full sessions ##########"
for d in $SETS; do
  n=$(basename "$d")
  /usr/bin/python3 ros_baseline.py "$d" --out "out/${n}_ros.json" 2>/dev/null | grep -v '^mono pinhole'
done

echo
echo "########## common-ground scoring (pose-only, all frames) ##########"
for d in $SETS; do
  n=$(basename "$d")
  python3 compare.py --cache "out/${n}_cache.json" --ros "out/${n}_ros.json" \
                     --rap "out/${n}_rap.json" --label "$n"
done

echo
echo "########## controlled holdout: C1 ROS-calib / C2 ours-M2 / C3 ours (Full) ##########"
python3 protocol.py $SETS --every 4 --json-out out/protocol.json 2>/dev/null \
  | grep -v '^mono pinhole'

echo
echo "########## rotating holdout + coverage audit ##########"
python3 kfold.py $SETS --every 4

echo
echo "########## final table (Table I): every method at its shipped defaults ##########"
if command -v mrcal-calibrate-cameras >/dev/null 2>&1; then
  python3 final_comparison.py $SETS --every 4 --json-out out/final_table.json 2>/dev/null \
    | grep -v '^mono pinhole'
else
  echo "  mrcal not installed (apt install mrcal); running without it"
  python3 final_comparison.py $SETS --every 4 --json-out out/final_table.json 2>/dev/null \
    | grep -v '^mono pinhole'
fi

echo
echo "########## fisheye probe (not in the paper) ##########"
if [ -f ../optional_mrcal_samyang_fisheye/corners.vnl ]; then
  python3 fisheye_probe.py --vnl ../optional_mrcal_samyang_fisheye/corners.vnl \
                           --images ../optional_mrcal_samyang_fisheye/images
else
  echo "  skipped: ../optional_mrcal_samyang_fisheye not fetched"
fi
