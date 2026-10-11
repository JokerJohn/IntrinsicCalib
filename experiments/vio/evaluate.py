#!/usr/bin/env python3
"""ATE / RPE of a trajectory against the LIO reference, using evo.

  evaluate.py --ref <lio.tum> --est <run.tum> [--json out.json] [--plot out.png] [--deltas 1 5]

By default the LIO reference is resampled (linear position / slerp) at the estimate timestamps, because
LIVO states (and OpenVINS poses) are stamped at image times, ~14 ms after the LiDAR scan stamps.
ATE: evo_ape, SE(3) alignment (-a, no scale), translation RMSE [m] and rotation RMSE [deg].
RPE: evo_rpe, delta = 1 m and 5 m of travelled path (all pairs), translation RMSE [m] and rotation RMSE [deg].
Timestamp association tolerance 0.02 s (both trajectories come from the same bag, LIO poses are at LiDAR
scan times; LIVO adds poses at image times, which are simply unmatched).
"""
import argparse, json, os, re, subprocess, sys
EVO = os.environ.get('EVO_BIN', os.path.expanduser('~/.local/bin'))   # directory with evo_ape / evo_rpe
def run(cmd):
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return p.returncode, p.stdout
def parse_stats(txt):
    s = {}
    for k in ('rmse', 'mean', 'median', 'std', 'min', 'max', 'sse'):
        m = re.search(r'^\s*%s\s+([0-9.eE+-]+)' % k, txt, re.M)
        if m: s[k] = float(m.group(1))
    return s
def count_matches(ref, est, tol):
    import numpy as np
    tr = np.loadtxt(ref, comments='#')[:, 0]; te = np.loadtxt(est, comments='#')[:, 0]
    te = np.sort(te); idx = np.searchsorted(te, tr); idx = np.clip(idx, 1, len(te) - 1)
    d = np.minimum(abs(te[idx] - tr), abs(te[idx - 1] - tr))
    return int((d <= tol).sum()), len(tr), len(te)
def interpolate_reference(ref, est, out):
    """Resample the reference trajectory at the estimate timestamps (linear position, slerp rotation).
    FAST-LIVO2 in LIVO mode and OpenVINS stamp their states at image times (~14 ms after the LiDAR scan
    stamps of the LIO reference); nearest-neighbour association would add up to ~15 mm of motion error."""
    import numpy as np
    from scipy.spatial.transform import Rotation, Slerp
    r = np.loadtxt(ref, comments='#'); e = np.loadtxt(est, comments='#')
    r = r[np.argsort(r[:, 0])]; _, keep = np.unique(r[:, 0], return_index=True); r = r[keep]
    t = e[:, 0]; t = t[(t >= r[0, 0]) & (t <= r[-1, 0])]
    pos = np.column_stack([np.interp(t, r[:, 0], r[:, k]) for k in (1, 2, 3)])
    q = Slerp(r[:, 0], Rotation.from_quat(r[:, 4:8]))(t).as_quat()
    np.savetxt(out, np.column_stack([t, pos, q]), fmt='%.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f')
    return len(t), len(e)
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ref', required=True); ap.add_argument('--est', required=True)
    ap.add_argument('--json'); ap.add_argument('--plot')
    ap.add_argument('--deltas', nargs='+', type=float, default=[1.0, 5.0], help='RPE deltas in metres')
    ap.add_argument('--t_max_diff', type=float, default=0.02)
    ap.add_argument('--skip_first', type=float, default=0.0, help='drop the first N seconds of the estimate (e.g. the VIO initialisation transient)')
    ap.add_argument('--no_interp', action='store_true', help='nearest-neighbour association instead of resampling the reference at the estimate timestamps')
    a = ap.parse_args()
    res = {'ref': a.ref, 'est': a.est, 'reference_interpolated': not a.no_interp, 'skip_first_s': a.skip_first}
    if a.skip_first > 0:
        import numpy as np
        e = np.loadtxt(a.est, comments='#'); e = e[e[:, 0] >= e[0, 0] + a.skip_first]
        cut = os.path.splitext(a.est)[0] + f'.skip{a.skip_first:g}.tum'; np.savetxt(cut, e, fmt='%.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f'); a.est = cut
    if not a.no_interp:
        ref_i = os.path.splitext(a.est)[0] + '.refinterp.tum'
        n_in, n_est = interpolate_reference(a.ref, a.est, ref_i)
        res['ref_interp_file'] = ref_i; a.ref = ref_i; a.t_max_diff = 0.001
    base = ['--t_max_diff', str(a.t_max_diff), '--no_warnings']
    rc, out = run([f'{EVO}/evo_ape', 'tum', a.ref, a.est, '-a', '-r', 'trans_part'] + base)
    if rc: print(out); sys.exit('evo_ape failed')
    res['ate_trans_m'] = parse_stats(out)
    res['matched'], res['n_ref'], res['n_est'] = count_matches(a.ref, a.est, a.t_max_diff)
    rc, out = run([f'{EVO}/evo_ape', 'tum', a.ref, a.est, '-a', '-r', 'angle_deg'] + base)
    res['ate_rot_deg'] = parse_stats(out)
    for d in a.deltas:
        for rel, key in (('trans_part', 'trans_m'), ('angle_deg', 'rot_deg')):
            rc, out = run([f'{EVO}/evo_rpe', 'tum', a.ref, a.est, '-r', rel, '--delta', str(d), '--delta_unit', 'm', '--all_pairs'] + base)
            res[f'rpe_{d:g}m_{key}'] = parse_stats(out) if rc == 0 else {'error': out.strip().splitlines()[-1]}
    if a.plot:
        env = dict(os.environ, MPLBACKEND='Agg')
        subprocess.run([f'{EVO}/evo_traj', 'tum', a.est, '--ref', a.ref, '-a', '--plot_mode', 'xy', '--save_plot', a.plot, '--no_warnings', '--t_max_diff', str(a.t_max_diff)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    g = lambda k, f='rmse': res.get(k, {}).get(f, float('nan'))
    print(f"ATE  trans RMSE {g('ate_trans_m'):.4f} m (mean {g('ate_trans_m','mean'):.4f}, max {g('ate_trans_m','max'):.4f}; "
          f"{res['matched']}/{res['n_ref']} ref poses matched) | rot RMSE {g('ate_rot_deg'):.3f} deg")
    for d in a.deltas:
        print(f"RPE@{d:g}m trans RMSE {g(f'rpe_{d:g}m_trans_m'):.4f} m | rot RMSE {g(f'rpe_{d:g}m_rot_deg'):.3f} deg "
              f"(all pairs, {res[f'rpe_{d:g}m_trans_m'].get('matched','')}max {g(f'rpe_{d:g}m_trans_m','max'):.4f} m)")
    if a.json:
        with open(a.json, 'w') as f: json.dump(res, f, indent=1)
if __name__ == '__main__':
    main()
