#!/usr/bin/env bash
# One-shot evaluation of ONE intrinsics set with all VIO baselines against the LIO references.
#   eval_set.sh <intrinsics.yaml> <set_name> [bag_key ...]
# bag_key: 2026-02-23-11-46-32 (27.5 s) | 2026-02-23-11-57-46 (97 s) | 2026-02-22-11-25-57 (72 s); default all three.
# Writes results/<set_name>/<bag_key>/{openvins_mono,openvins_stereo,vins_mono,vins_stereo}.tum + *_vs_lio.json
# and results/<set_name>/summary.md
set -uo pipefail
INTR=$(readlink -f "$1"); NAME=$2; shift 2
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Rig-C bags: $RIGC_DIR/<key>.bag (see data/rigc/MANIFEST.md for the identification of each file)
D=${RIGC_DIR:-$HERE/../../data/rigc}
declare -A BAGS=( [2026-02-23-11-46-32]=$D/2026-02-23-11-46-32.bag
                  [2026-02-23-11-57-46]=$D/2026-02-23-11-57-46.bag
                  [2026-02-22-11-25-57]=$D/2026-02-22-11-25-57.bag )
KEYS=("$@"); [ ${#KEYS[@]} -eq 0 ] && KEYS=(2026-02-23-11-46-32 2026-02-23-11-57-46 2026-02-22-11-25-57)
SET="$HERE/vio/configs/$NAME"; OUT="$HERE/results/$NAME"; mkdir -p "$OUT"
python3 "$HERE/vio/make_vio_config.py" "$INTR" "$SET" || exit 1
cp "$INTR" "$OUT/intrinsics.yaml"
for K in "${KEYS[@]}"; do
  BAG=${BAGS[$K]}; REF="$HERE/ref/${K}_lio.tum"; O="$OUT/$K"; mkdir -p "$O"
  [ -s "$REF" ] || { echo "missing reference $REF (run make_refs.sh)"; exit 1; }
  "$HERE/vio/run_openvins.sh" "$BAG" "$SET/openvins/estimator_config.yaml" "$O/openvins_mono.tum" mono
  "$HERE/vio/run_openvins.sh" "$BAG" "$SET/openvins/estimator_config.yaml" "$O/openvins_stereo.tum" stereo
  "$HERE/vio/run_vins.sh"     "$BAG" "$SET/vins/vins_mono_imu.yaml"   "$O/vins_mono.tum"
  "$HERE/vio/run_vins.sh"     "$BAG" "$SET/vins/vins_stereo_imu.yaml" "$O/vins_stereo.tum"
  for S in openvins_mono openvins_stereo vins_mono vins_stereo; do
    if [ -s "$O/$S.tum" ]; then
      echo "== $K $S"; "$HERE/evaluate.py" --ref "$REF" --est "$O/$S.tum" --json "$O/${S}_vs_lio.json" --plot "$O/${S}_vs_lio.png"
    else echo "== $K $S: NO TRAJECTORY"; fi
  done
done
python3 "$HERE/scripts/summarize.py" "$OUT" | tee "$OUT/summary.md"
