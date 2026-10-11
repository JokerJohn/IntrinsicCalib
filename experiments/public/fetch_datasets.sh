#!/usr/bin/env bash
# Fetch the five public chessboard sessions of Table I and lay each one out as
# a flat image folder that the calibration tool opens directly (Add Folder).
#
# Every session is a plain chessboard on a perspective camera, which is what
# the radial-order comparison (M2 vs M3) assumes.
#
# Usage:  ./fetch_datasets.sh [dataset ...]      (default: all)
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

have() { command -v "$1" >/dev/null 2>&1; }
have curl || { echo "curl required"; exit 1; }

# ---------------------------------------------------------------- opencv ----
# opencv/opencv samples/data — the stereo pair shipped with the library since
# the Learning-OpenCV book.  Note left10/right10 do not exist upstream.
fetch_opencv() {
  echo "== opencv (9x6) =="
  for side in left right; do
    d="opencv_$side"; mkdir -p "$d"
    for i in 01 02 03 04 05 06 07 08 09 11 12 13 14; do
      curl -fsSL --retry 3 -o "$d/$side$i.jpg" \
        "https://raw.githubusercontent.com/opencv/opencv/4.x/samples/data/$side$i.jpg" &
    done
    wait
    echo "   $d: $(ls "$d"/*.jpg | wc -l) images"
  done
  # reference intrinsics shipped alongside, for the Compare-with-YAML panel
  curl -fsSL --retry 3 -o opencv_left/reference_left_intrinsics.yml \
    "https://raw.githubusercontent.com/opencv/opencv/4.x/samples/data/left_intrinsics.yml"
}

# ------------------------------------------------------------- opencalib ----
# PJLab-ADG/SensorsCalibration (OpenCalib), camera_intrinsic module sample set:
# an automotive wide-angle camera, 1920x1200, visible barrel distortion.
fetch_opencalib() {
  echo "== opencalib (17x15) =="
  d=opencalib_front; mkdir -p "$d"
  base="https://raw.githubusercontent.com/PJLab-ADG/SensorsCalibration/master/camera_intrinsic/intrinsic_calib/data"
  for i in $(seq 2 23); do
    curl -fsSL --retry 3 -o "$d/$i.png" "$base/$i.png" &
  done
  wait
  find "$d" -name '*.png' -size -10k -delete   # drop any 404 body
  echo "   $d: $(ls "$d"/*.png | wc -l) images"
}

# --------------------------------------------------------------- ros wide ---
# The sample data behind the ROS camera_calibration tutorial (8x6, 108 mm) —
# the same node the paper compares against.  Shipped as .pgm, which the GUI
# loader does not accept, so convert to .png.
fetch_ros() {
  echo "== ros_wide (8x6, 108 mm) =="
  tmp=$(mktemp -d); trap 'rm -rf "$tmp"' RETURN
  curl -fsSL --retry 3 -o "$tmp/cc.tar.gz" \
    "http://download.ros.org/data/camera_calibration/camera_calibration.tar.gz" || return 1
  tar xzf "$tmp/cc.tar.gz" -C "$tmp"
  for side in left right; do
    d="ros_wide_$side"; mkdir -p "$d"
    python3 - "$tmp/wide" "$d" "$side" <<'PY'
import sys, glob, os, cv2
src, dst, side = sys.argv[1:4]
for p in sorted(glob.glob(os.path.join(src, side + "*.pgm"))):
    im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
    cv2.imwrite(os.path.join(dst, os.path.basename(p)[:-4] + ".png"), im)
PY
    echo "   $d: $(ls "$d"/*.png | wc -l) images (pgm -> png)"
  done
}

case "${1:-all}" in
  all)       fetch_opencv; fetch_opencalib; fetch_ros ;;
  opencv)    fetch_opencv ;;
  opencalib) fetch_opencalib ;;
  ros)       fetch_ros ;;
  *)         echo "unknown: $1 (all|opencv|opencalib|ros)"; exit 2 ;;
esac

echo
echo "Now write the GUI session files:  ./write_sessions.sh"
