#!/usr/bin/env python3
"""Rig-D (Rig-A camera) intrinsics comparison with OpenVINS mono (deterministic serial reader).
  run_rigd.py run  [--sets ...]     rectify -> config -> OpenVINS -> delete the rectified bag (resumable)
  run_rigd.py eval [--sets ...]     evaluate every run against the reference set, write rigd_results_gt.{json,md}
  --ref-set gt (default)           ground truth, see ../refs.py; the runs are scored on copies in runs_gt/ (runs/ is not touched)
  --ref-set lio                    earlier FAST-LIVO2 reference ../ref/rigd_office1_lio.tum, scored in place -> rigd_results.{json,md}
  --rescore-only                   same as the stage 'eval' (re-scores the existing trajectories, no VIO run)
Per set: rect/<set>_rect.bag (scripts/make_rect_bag_rigd.py, full distortion model of the set, mono8, 960x540),
configs/<set>/openvins (scripts/make_vio_config_rigd.py, fx fy cx cy of the set scaled by 0.5, zero distortion),
runs/<set>/openvins_mono.{tum,log}. Everything except the four pinhole numbers is identical across sets.
"""
import argparse, json, os, shutil, subprocess, sys, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, f'{HERE}/..')
import refs
SEQ = 'rigd_office1'
RAW = f'{HERE}/bags/rigd_office1_raw.bag'; REF = refs.ref_path('lio', SEQ)   # REF: lio reference; use ref_for(ref_set)
TD = 0.005   # camera-IMU time shift (t_imu = t_cam + TD), scripts/td_yaw_fit.py, fixed for every set
SETS = ['ROS-calib', 'mrcal', 'Kalibr', 'ours-M2', 'ours', 'Kalibr-run2']
LABEL = {'ROS-calib': 'ROS-calib', 'mrcal': 'mrcal', 'Kalibr': 'Kalibr', 'ours-M2': 'ours-M2', 'ours': 'ours (Full)', 'Kalibr-run2': 'Kalibr (repeat run)'}
def sh(cmd, log=None):
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if log: open(log, 'a').write(p.stdout)
    return p.returncode, p.stdout
def ref_for(ref_set): return refs.ref_path(ref_set, SEQ)
def score_run(tum, ref_set, name=None):
    """Score one trajectory against `ref_set`; returns the result dict (evaluate.py json + traj_stats) or {'error': ...}.
    lio: in place (<tum>_vs_lio.json, plot), as before; gt: on a copy runs_gt/<set>/<name>.tum."""
    ref = ref_for(ref_set)
    if ref_set == 'lio':
        js = tum[:-4] + '_vs_lio.json'; est = tum; extra = f'--plot {tum[:-4]}_vs_lio.png'
    else:
        d = f'{HERE}/runs_{ref_set}/{os.path.basename(os.path.dirname(tum))}'; os.makedirs(d, exist_ok=True)
        est = f'{d}/{os.path.basename(tum)}'; shutil.copyfile(tum, est); js = est[:-4] + f'_vs_{ref_set}.json'; extra = ''
    rc, o = sh(f'timeout 600 {HERE}/../evaluate.py --ref {ref} --est {est} --json {js} {extra}')
    if rc: return {'error': o[-300:]}
    r = json.load(open(js)); r.update(traj_stats(tum, ref)); return r
def traj_stats(tum, ref):
    e = np.loadtxt(tum); r = np.loadtxt(ref)
    if e.ndim != 2 or len(e) < 2: return {'poses': 0, 'tracked_to_end': False}
    seg = np.linalg.norm(np.diff(e[:, 1:4], axis=0), axis=1); dt = np.diff(e[:, 0]); v = seg[dt > 1e-6] / dt[dt > 1e-6]
    rseg = np.linalg.norm(np.diff(r[:, 1:4], axis=0), axis=1).sum()
    # coverage of the MOVING part of the reference (the filter initialises at motion onset, ~10 s)
    rv = np.linalg.norm(np.diff(r[:, 1:4], axis=0), axis=1) / np.diff(r[:, 0]); mv = r[1:, 0][rv > 0.05]
    t_last_ref_motion = mv[-1]; t_first_ref_motion = mv[0]
    ok_end = e[-1, 0] >= t_last_ref_motion - 1.0
    ratio = seg.sum() / rseg
    return {'poses': int(len(e)), 't_first_s': float(e[0, 0] - r[0, 0]), 't_last_s': float(e[-1, 0] - r[0, 0]),
            'ref_motion_s': [float(t_first_ref_motion - r[0, 0]), float(t_last_ref_motion - r[0, 0])],
            'path_m': float(seg.sum()), 'ref_path_m': float(rseg), 'path_ratio': float(ratio), 'max_speed_mps': float(v.max()),
            'tracked_to_end': bool(ok_end and 0.5 <= ratio <= 2.0 and v.max() < 5.0)}
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('stage', choices=['run', 'eval'], nargs='?'); ap.add_argument('--sets', nargs='+', default=SETS)
    ap.add_argument('--keep_rect', action='store_true'); refs.add_args(ap); a = ap.parse_args()
    if a.rescore_only: a.stage = 'eval'
    if a.stage is None: ap.error('stage (run | eval) or --rescore-only required')
    sfx = '' if a.ref_set == 'lio' else f'_{a.ref_set}'
    for d in ('logs', 'rect', 'runs', 'configs'): os.makedirs(f'{HERE}/{d}', exist_ok=True)
    if a.stage == 'run':
        for st in a.sets:
            od = f'{HERE}/runs/{st}'; os.makedirs(od, exist_ok=True); tum = f'{od}/openvins_mono.tum'
            if os.path.exists(tum) and os.path.getsize(tum) > 0: print('[rigd] exists', tum, flush=True); continue
            rect = f'{HERE}/rect/{st}_rect.bag'; lg = f'{HERE}/logs/run_{st}.console'
            if not os.path.exists(rect):
                rc, o = sh(f'timeout 1800 python3 {HERE}/scripts/make_rect_bag_rigd.py {RAW} {HERE}/intrinsics/{st}.yaml {rect}.tmp', lg)
                if rc: print('[rigd] rectification failed', st, o[-400:], flush=True); continue
                os.rename(f'{rect}.tmp', rect); os.rename(f'{rect}.tmp.K.yaml', f'{rect}.K.yaml')
            sh(f'cp {rect}.K.yaml {od}/rect_K.yaml')
            rc, o = sh(f'python3 {HERE}/scripts/make_vio_config_rigd.py {rect}.K.yaml {HERE}/configs/{st} --td {TD}', lg)
            rc, o = sh(f'{HERE}/run_openvins_rigd.sh {rect} {HERE}/configs/{st}/openvins/estimator_config.yaml {tum}', lg)
            print(f'[rigd] {st}:', o.strip().splitlines()[-1] if o.strip() else 'no output', flush=True)
            if not a.keep_rect and os.path.exists(tum) and os.path.getsize(tum) > 0:
                os.remove(rect); print('[rigd] deleted', rect, flush=True)
        return
    R = {'reference': ref_for(a.ref_set), 'ref_set': a.ref_set, 'td_s': TD, 'runs': {}}
    for st in a.sets:
        tum = f'{HERE}/runs/{st}/openvins_mono.tum'; res = {'label': LABEL[st], 'tum': tum}
        if os.path.exists(tum) and os.path.getsize(tum) > 0:
            res.update(score_run(tum, a.ref_set))
        else: res.update({'error': 'no trajectory', 'tracked_to_end': False})
        R['runs'][st] = res
    json.dump(R, open(f'{HERE}/rigd_results{sfx}.json', 'w'), indent=1)
    g = lambda r, k: r.get(k, {}).get('rmse', float('nan'))
    L = ['# Rig-D (Rig-A camera model): OpenVINS mono + IMU on rigd_office1 (163 s, 126 m) vs ' + ('ground truth' if a.ref_set == 'gt' else 'FAST-LIVO2 LIO reference') + '', '',
         'ATE: SE(3) Umeyama alignment (no scale), translation RMSE. RPE: translation RMSE over all pairs 1 m / 5 m of travelled path. '
         'Images rectified with the full model of each set, 960x540, zero distortion in OpenVINS; extrinsics, time shift, IMU noise and estimator settings identical. '
         'tracked = output until the end of the robot motion, path ratio in [0.5, 2], max speed < 5 m/s.', '',
         '| intrinsics | ATE [m] | RPE@1m [m] | RPE@5m [m] | tracked |', '|---|---|---|---|---|']
    for st in a.sets:
        r = R['runs'][st]
        if st == 'Kalibr-run2': L += ['', 'Kalibr repeat (second Kalibr run on the same 39 images, random view selection; not a row of the comparison):', '', '| intrinsics | ATE [m] | RPE@1m [m] | RPE@5m [m] | tracked |', '|---|---|---|---|---|']
        if 'ate_trans_m' not in r: L.append(f"| {r['label']} | - | - | - | no ({r.get('error','')[:40]}) |"); continue
        L.append(f"| {r['label']} | {g(r,'ate_trans_m'):.3f} | {g(r,'rpe_1m_trans_m'):.3f} | {g(r,'rpe_5m_trans_m'):.3f} | {'yes' if r['tracked_to_end'] else 'no'} (path {r['path_m']:.1f}/{r['ref_path_m']:.1f} m) |")
    open(f'{HERE}/rigd_results{sfx}.md', 'w').write('\n'.join(L) + '\n'); print('\n'.join(L))
if __name__ == '__main__':
    main()
