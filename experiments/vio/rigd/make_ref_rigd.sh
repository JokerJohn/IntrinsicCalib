#!/usr/bin/env bash
# LIO reference for the Rig-D sequence rigd_office1 -> ../ref/rigd_office1_lio.tum (run twice, must be bit-identical).
# Input bags (made once): scripts/split_bag.py <orig.bag> bags ; scripts/sanitize_lio_bag.py (drops 48 points with corrupted stamps).
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$HERE/ref_check" "$HERE/results"
"$HERE/run_lio.sh" "$HERE/bags/rigd_office1_lio.bag" "$HERE/config/rigd_lio.yaml" "$HERE/config/camera_dummy.yaml" "$HERE/ref_check/ref_run1.tum" 0.5 || exit 1
"$HERE/run_lio.sh" "$HERE/bags/rigd_office1_lio.bag" "$HERE/config/rigd_lio.yaml" "$HERE/config/camera_dummy.yaml" "$HERE/ref_check/ref_run2.tum" 0.5 || exit 1
cmp "$HERE/ref_check/ref_run1.tum" "$HERE/ref_check/ref_run2.tum" && echo "deterministic" && cp "$HERE/ref_check/ref_run1.tum" "$HERE/../ref/rigd_office1_lio.tum"
python3 "$HERE/scripts/loop_error.py" "$HERE/results/lio_loop_closure_check.json" "$HERE/../ref/rigd_office1_lio.tum"
