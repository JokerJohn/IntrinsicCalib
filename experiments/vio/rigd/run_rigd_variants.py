#!/usr/bin/env python3
"""Sensitivity check: is the single-run ranking of run_rigd.py larger than the run-to-run sensitivity of mono VIO?
Every set is rerun with four front-end variants (identical for all sets; the base run of run_rigd.py is variant v0):
  v1 num_pts 200, v2 num_pts 300, v3 fast_threshold 15, v4 fast_threshold 25   (base: num_pts 250, fast_threshold 20)
  run_rigd_variants.py run [--sets ...]   (rectified bag made once per set, 4 variants in parallel, bag deleted afterwards)
  run_rigd_variants.py eval               -> rigd_variants_gt.json / .md (median and range over v0..v4 per set), --ref-set gt (default)
  --ref-set lio                          earlier FAST-LIVO2 reference (in place) -> rigd_variants.json / .md
  --rescore-only                         same as the stage 'eval' (re-scores the existing trajectories, no VIO run)
"""
import argparse, json, os, re, shutil, subprocess, sys, numpy as np
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import run_rigd as B
V = {'v1': [('num_pts: 250', 'num_pts: 200')], 'v2': [('num_pts: 250', 'num_pts: 300')],
     'v3': [('fast_threshold: 20', 'fast_threshold: 15')], 'v4': [('fast_threshold: 20', 'fast_threshold: 25')]}
def main():
    ap = argparse.ArgumentParser(); ap.add_argument('stage', choices=['run', 'eval'], nargs='?'); ap.add_argument('--sets', nargs='+', default=B.SETS)
    B.refs.add_args(ap); a = ap.parse_args()
    if a.rescore_only: a.stage = 'eval'
    if a.stage is None: ap.error('stage (run | eval) or --rescore-only required')
    sfx = '' if a.ref_set == 'lio' else f'_{a.ref_set}'
    for d in ('rect', 'runs', 'configs'): os.makedirs(f'{HERE}/{d}', exist_ok=True)
    for st in a.sets: os.makedirs(f'{HERE}/runs/{st}', exist_ok=True)
    if a.stage == 'run':
        for st in a.sets:
            todo = [v for v in V if not os.path.exists(f'{HERE}/runs/{st}/openvins_mono_{v}.tum')]
            if not todo: continue
            rect = f'{HERE}/rect/{st}_rect.bag'
            if not os.path.exists(rect):
                rc, o = B.sh(f'timeout 1800 python3 {HERE}/scripts/make_rect_bag_rigd.py {B.RAW} {HERE}/intrinsics/{st}.yaml {rect}.tmp')
                os.rename(f'{rect}.tmp', rect); os.rename(f'{rect}.tmp.K.yaml', f'{rect}.K.yaml')
            for v in todo:
                d = f'{HERE}/configs/{st}_{v}'
                B.sh(f'python3 {HERE}/scripts/make_vio_config_rigd.py {rect}.K.yaml {d} --td {B.TD}')
                p = f'{d}/openvins/estimator_config.yaml'; s = open(p).read()
                for x, y in V[v]: assert x in s; s = s.replace(x, y + f'   # variant {v}', 1)
                open(p, 'w').write(s)
            with ThreadPoolExecutor(4) as ex:
                outs = list(ex.map(lambda v: B.sh(f'{HERE}/run_openvins_rigd.sh {rect} {HERE}/configs/{st}_{v}/openvins/estimator_config.yaml {HERE}/runs/{st}/openvins_mono_{v}.tum'), todo))
            for v, (rc, o) in zip(todo, outs): print(f'[var] {st} {v}:', o.strip().splitlines()[-1] if o.strip() else '-', flush=True)
            os.remove(rect); print('[var] deleted', rect, flush=True)
        return
    R = {}
    for st in a.sets:
        R[st] = {}
        for v in ['v0'] + list(V):
            tum = f'{HERE}/runs/{st}/openvins_mono.tum' if v == 'v0' else f'{HERE}/runs/{st}/openvins_mono_{v}.tum'
            if not os.path.exists(tum) or os.path.getsize(tum) == 0: R[st][v] = {'error': 'no trajectory'}; continue
            if a.ref_set == 'lio':   # old behaviour: reuse the stored v0 score of run_rigd.py eval
                js = tum[:-4] + '_vs_lio.json'
                if v != 'v0' or not os.path.exists(js): B.sh(f'timeout 600 {HERE}/../evaluate.py --ref {B.REF} --est {tum} --json {js}')
                r = json.load(open(js)); r.update(B.traj_stats(tum, B.REF))
            else: r = B.score_run(tum, a.ref_set)
            R[st][v] = r
    json.dump(R, open(f'{HERE}/rigd_variants{sfx}.json', 'w'), indent=1)
    L = ['# Front-end sensitivity' + (' (ground-truth reference)' if a.ref_set == 'gt' else '') + ': 5 runs per set (v0 = base; v1/v2 num_pts 200/300; v3/v4 fast_threshold 15/25), identical for all sets', '',
         'median [min-max] over the 5 runs; tracked = runs tracked to the end (run_rigd.py criterion)', '',
         '| intrinsics | ATE [m] | RPE@1m [m] | RPE@5m [m] | tracked | per-run ATE v0..v4 [m] |', '|---|---|---|---|---|---|']
    for st in a.sets:
        rr = [R[st][v] for v in ['v0'] + list(V)]
        def col(k):
            x = np.array([r.get(k, {}).get('rmse', np.nan) for r in rr]); return f'{np.nanmedian(x):.2f} [{np.nanmin(x):.2f}-{np.nanmax(x):.2f}]'
        ate = ' '.join(f"{r.get('ate_trans_m', {}).get('rmse', float('nan')):.2f}" for r in rr)
        L.append(f"| {B.LABEL[st]} | {col('ate_trans_m')} | {col('rpe_1m_trans_m')} | {col('rpe_5m_trans_m')} | {sum(bool(r.get('tracked_to_end')) for r in rr)}/5 | {ate} |")
    open(f'{HERE}/rigd_variants{sfx}.md', 'w').write('\n'.join(L) + '\n'); print('\n'.join(L))
if __name__ == '__main__':
    main()
