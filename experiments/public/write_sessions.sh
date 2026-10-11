#!/usr/bin/env bash
# Drop a .fastcalib_intrinsics.json into every fetched folder so the
# calibration tool opens it with the right board already filled in.
#
# The geometries below are not guesses: cols/rows were confirmed by detecting
# the full inner-corner grid on every image of each set (see verify_datasets.py
# and README.md).  Square size only sets the extrinsic scale — it
# does not move K or the distortion coefficients — so where the upstream source
# does not state it, a nominal value is used and noted in README.md.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
MK="python3 $ROOT/make_gui_session.py"

s() {  # s <folder> <cols> <rows> <square_mm> [extra flags...]
  local d=$1; shift
  [ -d "$d" ] || { echo "   skip $d (not fetched)"; return; }
  $MK "$d" --cols "$1" --rows "$2" --square-mm "$3" "${@:4}"
}

s opencv_left          9  6  30.0
s opencv_right         9  6  30.0
s opencalib_front     17 15  50.0
s ros_wide_left        8  6 108.0
s ros_wide_right       8  6 108.0

# The optional mrcal fisheye sessions receive no session file: both OpenCV
# detectors fail to recover the 14x14 grid on these images.
