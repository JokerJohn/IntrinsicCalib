#!/usr/bin/env python3
"""Table III: trajectory error of OpenVINS on the Rig-C and Rig-D sequences.

Scores the shipped OpenVINS trajectories (results/trajectories/) against the
ground truth (gt/) with evaluate.py (reference resampled at the estimate
stamps; ATE: SE(3)-aligned translation RMSE; RPE: translation RMSE over all
pairs 1 m apart).  Rig-C: stereo OpenVINS, one run per intrinsics set and
sequence.  Rig-D: monocular OpenVINS, five front-end settings v0..v4 per set;
the table entry is the median over the five runs.

  make_table3.py [--traj results/trajectories] [--json results/table3.json]

The trajectories are the outputs of run_matrix.py (Rig-C, file
results/matrix/<set>/<sequence>/openvins_stereo.tum) and of rigd/run_rigd.py
and rigd/run_rigd_variants.py (Rig-D, rigd/runs/<set>/openvins_mono[_v1..v4].tum);
re-running the VIO requires ROS, OpenVINS and the bags (README.md).
Requires evo (evo_ape, evo_rpe; directory in EVO_BIN).
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GT = {'S1': 'gt_rigc_s1.txt', 'S2': 'gt_rigc_s2.txt', 'S3': 'gt_rigc_s3.txt', 'Rig-D': 'gt_rigd_office1.txt'}
# (row label of Table III, directory under results/trajectories/<platform>/)
RIGC_ROWS = [('Kalibr', 'Kalibr'), ('ours-M2', 'ours-M2'), ('ours (Full)', 'ours')]
RIGD_ROWS = [('ROS-calib', 'ROS-calib'), ('mrcal', 'mrcal'), ('Kalibr', 'Kalibr'),
             ('ours-M2', 'ours-M2'), ('ours (Full)', 'ours')]
VARIANTS = ['v0', 'v1', 'v2', 'v3', 'v4']


def score(est, ref, tmp):
    """ATE and RPE@1m [m] of one trajectory (evaluate.py writes side files next to the estimate)."""
    d = tempfile.mkdtemp(dir=tmp)
    e = os.path.join(d, 'est.tum')
    shutil.copyfile(est, e)
    js = os.path.join(d, 'result.json')
    p = subprocess.run([sys.executable, os.path.join(HERE, 'evaluate.py'), '--ref', ref, '--est', e,
                        '--json', js, '--deltas', '1'], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError((p.stdout + p.stderr)[-500:])
    r = json.load(open(js))
    return {'ate_m': r['ate_trans_m']['rmse'], 'rpe1_m': r['rpe_1m_trans_m']['rmse']}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--traj', default=os.path.join(HERE, 'results', 'trajectories'))
    ap.add_argument('--json', default=os.path.join(HERE, 'results', 'table3.json'))
    a = ap.parse_args()
    gt = lambda k: os.path.join(HERE, 'gt', GT[k])
    out = {'rigc': {}, 'rigd': {}}
    with tempfile.TemporaryDirectory() as tmp:
        for lab, d in RIGC_ROWS:
            out['rigc'][lab] = {s: score(os.path.join(a.traj, 'rigc', d, f'{s}.tum'), gt(s), tmp)
                                for s in ('S1', 'S2', 'S3')}
        for lab, d in RIGD_ROWS:
            runs = {v: score(os.path.join(a.traj, 'rigd', d, f'{v}.tum'), gt('Rig-D'), tmp) for v in VARIANTS}
            med = {k: float(np.median([runs[v][k] for v in VARIANTS])) for k in ('ate_m', 'rpe1_m')}
            out['rigd'][lab] = {'runs': runs, 'median': med}
    json.dump(out, open(a.json, 'w'), indent=1)

    print('Table III, OpenVINS trajectory error [m] (ATE / RPE@1m); Rig-D: median of five front-end settings')
    print(f"{'method':12s} {'Rig-C S1':>15s} {'Rig-C S2':>15s} {'Rig-C S3':>15s} {'Rig-D':>13s}")
    for lab in ['ROS-calib', 'mrcal', 'Kalibr', 'ours-M2', 'ours (Full)']:
        c = out['rigc'].get(lab)
        cells = [f"{c[s]['ate_m']:.3f} / {c[s]['rpe1_m']:.3f}" if c else 'x' for s in ('S1', 'S2', 'S3')]
        m = out['rigd'][lab]['median']
        cells.append(f"{m['ate_m']:.2f} / {m['rpe1_m']:.2f}")
        print(f"{lab:12s} " + ' '.join(f'{x:>15s}' for x in cells[:3]) + f' {cells[3]:>13s}')
    print('Rig-D ATE range over v0..v4 [m]: ' + ', '.join(
        f"{lab} {min(r['ate_m'] for r in out['rigd'][lab]['runs'].values()):.2f}-"
        f"{max(r['ate_m'] for r in out['rigd'][lab]['runs'].values()):.2f}" for lab, _ in RIGD_ROWS))


if __name__ == '__main__':
    main()
