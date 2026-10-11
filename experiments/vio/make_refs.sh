#!/usr/bin/env bash
# Produce the FAST-LIVO2 LIO-only reference trajectories for the three SLAM bags -> ref/<bag>_lio.tum
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Rig-C bags: $RIGC_DIR/<key>.bag (see data/rigc/MANIFEST.md for the identification of each file)
D=${RIGC_DIR:-$HERE/../../data/rigc}
for BAG in \
  $D/2026-02-23-11-46-32.bag \
  $D/2026-02-23-11-57-46.bag \
  $D/2026-02-22-11-25-57.bag ; do
  NAME=$(basename "$BAG" .bag)
  "$HERE/run_one.sh" "$BAG" "$HERE/config/flir_lio.yaml" "$HERE/config/camera_flir_cam0.yaml" "$HERE/ref/${NAME}_lio.tum" 0.5 || exit 1   # half rate: FAST-LIVO2 is slower than real time on 262k-pt scans and roslaunch stops it when the bag ends
done
