#!/usr/bin/env python3
"""Intrinsics-set x bag x system matrix with pre-rectified images (see README "Pre-rectified evaluation").

  run_matrix.py [--sets RAP kalibr M2-R] [--systems openvins_stereo] [--bags ...]
                [--wait_min 90] [--min_free_gb 12] [--rescore-only]

For each set (in order) and each bag: waits until cam0/cam1 files of the bag's session exist in
experiments/rigc/intrinsics/ (D1 for the 02-22 bag, D2 for the 02-23 bags), makes/caches the rectified VIO bag in
flir_eval_rawbags/<set>/<key>_vio_rect.bag, generates vio/configs/<set>_<session>/ (zero distortion),
runs the systems, evaluates against the reference set, stores results/matrix/<set>/<key>/<system>.tum and writes
the summary after every run (resumable: existing .tum files are reused).
Scores against the ground truth (refs.py) -> results/matrix_gt.json + matrix_gt.md; the trajectories are scored on
copies in results/matrix_gt/<set>/<key>/<system>/, results/matrix/ is not touched by the scoring.
  --rescore-only           score the existing trajectories only (no waiting, rectification or VIO run)
Sets: kalibr, M2-R and RAP = the rows Kalibr, ours-M2 and ours (Full) of Table III (Rig-C, stereo OpenVINS).
"""
import argparse, json, os, shutil, subprocess, sys, time, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import refs
DATA = os.environ.get('RIGC_DIR', os.path.join(HERE, '..', '..', 'data', 'rigc'))   # Rig-C bags
INTR = os.path.join(HERE, '..', 'rigc', 'intrinsics')                                # written by experiments/rigc/flir_full.py
RAW = f'{DATA}/flir_eval_rawbags'                                                    # derived bags (generated here)
BAGS = {'2026-02-23-11-46-32': 'D2', '2026-02-23-11-57-46': 'D2', '2026-02-22-11-25-57': 'D1'}
def sh(cmd, log=None):
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if log: open(log, 'a').write(p.stdout)
    return p.returncode, p.stdout
def free_gb(path): s = os.statvfs(path); return s.f_bavail * s.f_frsize / 1e9
def score(tum, ref, ref_set, st, b, sysn):
    """evaluate.py of one trajectory on a copy; returns (returncode, output, json path)."""
    d = f'{HERE}/results/matrix_{ref_set}/{st}/{b}/{sysn}'; os.makedirs(d, exist_ok=True)
    shutil.copyfile(tum, f'{d}/est.tum'); js = f'{d}/result.json'     # evaluate.py writes est.refinterp.tum next to the estimate
    rc, o = sh(f'{HERE}/evaluate.py --ref {ref} --est {d}/est.tum --json {js}')
    return rc, o, js
def traj_stats(tum, ref):
    e = np.loadtxt(tum, comments='#'); r = np.loadtxt(ref, comments='#')
    if e.ndim != 2 or len(e) < 2: return {'poses': 0, 'tracked_to_end': False}
    seg = np.linalg.norm(np.diff(e[:, 1:4], axis=0), axis=1); dt = np.diff(e[:, 0]); v = seg[dt > 1e-6] / dt[dt > 1e-6]
    rseg = np.linalg.norm(np.diff(r[:, 1:4], axis=0), axis=1).sum()
    cov = (e[-1, 0] - e[0, 0]) / (r[-1, 0] - r[0, 0]); ratio = seg.sum() / rseg
    return {'poses': int(len(e)), 'duration_s': float(e[-1, 0] - e[0, 0]), 'path_m': float(seg.sum()), 'ref_path_m': float(rseg),
            'path_ratio': float(ratio), 'max_speed_mps': float(v.max()), 'time_coverage': float(cov),
            'tracked_to_end': bool(cov >= 0.9 and 0.5 <= ratio <= 2.0 and v.max() < 5.0)}
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sets', nargs='+', default=['RAP', 'kalibr', 'M2-R'])
    ap.add_argument('--systems', nargs='+', default=['openvins_stereo'], choices=['openvins_stereo', 'openvins_mono'])
    ap.add_argument('--bags', nargs='+', default=list(BAGS))
    ap.add_argument('--md', default=None, help='markdown summary path (default results/matrix_gt.md; only the --sets/--systems given are written)')
    ap.add_argument('--wait_min', type=float, default=90); ap.add_argument('--min_free_gb', type=float, default=12)
    refs.add_args(ap); a = ap.parse_args()
    out = f'{HERE}/results/matrix'; os.makedirs(out, exist_ok=True)
    sfx = f'_{a.ref_set}'
    mj = f'{HERE}/results/matrix{sfx}.json'
    if a.md is None: a.md = f'{HERE}/results/matrix{sfx}.md'
    if a.rescore_only: a.wait_min = 0
    M = json.load(open(mj)) if os.path.exists(mj) else {}
    def save():
        json.dump(M, open(mj, 'w'), indent=1)
        with open(a.md, 'w') as f:
            f.write('# Intrinsics-set matrix (pre-rectified images, zero distortion in the VIO configs)'
                    + ', reference set: ground truth (gt)\n\n'
                    'Cell = ATE trans RMSE [m] / RPE@1m trans RMSE [m]; "x" = did not track to the end (coverage < 90 %, path ratio outside 0.5-2 or speed > 5 m/s); "-" = not run.\n\n')
            for sysn in a.systems:
                f.write(f'## {sysn}\n\n| set | ' + ' | '.join(a.bags) + ' |\n|---|' + '---|' * len(a.bags) + '\n')
                for st in a.sets:
                    cells = []
                    for b in a.bags:
                        r = M.get(st, {}).get(b, {}).get(sysn)
                        if not r or 'ate_trans_m' not in r: cells.append('-'); continue
                        c = f"{r['ate_trans_m']['rmse']:.3f} / {r['rpe_1m_trans_m']['rmse']:.3f}"
                        cells.append(c if r['tracked_to_end'] else c + ' x')
                    f.write(f'| {st} | ' + ' | '.join(cells) + ' |\n')
                f.write('\n')
    for st in a.sets:
        for b in a.bags:
            ses = BAGS[b]; f0 = f'{INTR}/cam0_{ses}_{st}.yaml'; f1 = f'{INTR}/cam1_{ses}_{st}.yaml'
            t0 = time.time()
            if a.rescore_only:
                run_dir = f'{out}/{st}/{b}'
                if not os.path.isdir(run_dir): continue
                ref = refs.ref_path(a.ref_set, b)
                M.setdefault(st, {}).setdefault(b, {})
                M[st][b]['reference'] = ref
                for sysn in a.systems:
                    tum = f'{run_dir}/{sysn}.tum'; res = {'tum': tum}
                    if os.path.exists(tum) and os.path.getsize(tum) > 0:
                        rc, o, js = score(tum, ref, a.ref_set, st, b, sysn)
                        if rc == 0 and os.path.exists(js):
                            res.update(json.load(open(js))); res.update(traj_stats(tum, ref))
                            print(f'[matrix] {st} {b} {sysn}: ATE {res["ate_trans_m"]["rmse"]:.3f} m RPE1 {res["rpe_1m_trans_m"]["rmse"]:.3f} m tracked={res["tracked_to_end"]}', flush=True)
                        else: res['error'] = o[-300:]; print('[matrix] evaluate failed', o[-300:], flush=True)
                    else: continue
                    M[st][b][sysn] = res
                save(); continue
            while not (os.path.exists(f0) and os.path.exists(f1)):
                if time.time() - t0 > a.wait_min * 60: print(f'[matrix] TIMEOUT waiting for {f0} / {f1}', flush=True); break
                print(f'[matrix] waiting for {os.path.basename(f0)} / {os.path.basename(f1)} ...', flush=True); time.sleep(60)
            if not (os.path.exists(f0) and os.path.exists(f1)): continue
            cfg = f'{HERE}/vio/configs/{st}_{ses}'
            if not os.path.exists(f'{cfg}/openvins/kalibr_imucam_chain.yaml'):
                rc, o = sh(f'{HERE}/vio/make_vio_config.py --cam0 {f0} --cam1 {f1} --rectified {cfg}'); print(o.strip(), flush=True)
                if rc: print('[matrix] config generation failed', flush=True); continue
            rect = f'{RAW}/{st}/{b}_vio_rect.bag'
            if not os.path.exists(rect):
                if free_gb(RAW) < a.min_free_gb: print(f'[matrix] STOP: only {free_gb(RAW):.1f} GB free', flush=True); save(); return
                os.makedirs(os.path.dirname(rect), exist_ok=True)
                print(f'[matrix] rectifying {b} with set {st} ({ses}) ...', flush=True)
                rc, o = sh(f'python3 {HERE}/scripts/make_rect_bag.py {RAW}/{b}_vio.bag {f0} {f1} {rect}.tmp && mv {rect}.tmp {rect}')
                print(o.strip().splitlines()[-1] if o.strip() else '', flush=True)
                if rc: print('[matrix] rectification failed', o[-500:], flush=True); continue
            M.setdefault(st, {}).setdefault(b, {})['reference'] = refs.ref_path(a.ref_set, b)
            M.setdefault(st, {}).setdefault(b, {})['intrinsics'] = {'cam0': f0, 'cam1': f1, 'session': ses}
            ref = refs.ref_path(a.ref_set, b); od = f'{out}/{st}/{b}'; os.makedirs(od, exist_ok=True)
            for sysn in a.systems:
                tum = f'{od}/{sysn}.tum'
                if not os.path.exists(tum) or os.path.getsize(tum) == 0:
                    print(f'[matrix] run {st} {b} {sysn}', flush=True)
                    mode = sysn.split('_')[1]
                    rc, o = sh(f'{HERE}/vio/run_openvins.sh {rect} {cfg}/openvins/estimator_config.yaml {tum} {mode} serial')
                    print(o.strip().splitlines()[-1] if o.strip() else '', flush=True)
                res = {'tum': tum}
                if os.path.exists(tum) and os.path.getsize(tum) > 0:
                    rc, o, js = score(tum, ref, a.ref_set, st, b, sysn)
                    if rc == 0 and os.path.exists(js):
                        res.update(json.load(open(js))); res.update(traj_stats(tum, ref))
                        print(f'[matrix] {st} {b} {sysn}: ATE {res["ate_trans_m"]["rmse"]:.3f} m RPE1 {res["rpe_1m_trans_m"]["rmse"]:.3f} m tracked={res["tracked_to_end"]}', flush=True)
                    else: res['error'] = o[-300:]; print('[matrix] evaluate failed', o[-300:], flush=True)
                else: res['error'] = 'no trajectory'; print('[matrix] no trajectory', flush=True)
                M[st][b][sysn] = res; save()
    save(); print('[matrix] done', flush=True)
if __name__ == '__main__':
    main()
