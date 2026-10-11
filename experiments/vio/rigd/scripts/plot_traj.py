#!/usr/bin/env python3
"""plot_traj.py <ref.tum> <out.png> <est1.tum> [est2.tum ...]: SE(3)-aligned (Umeyama, no scale) xy plot + speed vs time."""
import sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
ref = np.loadtxt(sys.argv[1]); out = sys.argv[2]
fig, ax = plt.subplots(1, 2, figsize=(15, 6)); ax[0].plot(ref[:, 1], ref[:, 2], 'k', lw=2, label='LIO ref')
tr = ref[:, 0]; vr = np.linalg.norm(np.gradient(ref[:, 1:4], tr, axis=0), axis=1); ax[1].plot(tr - tr[0], vr, 'k', lw=2, label='ref')
for f in sys.argv[3:]:
    e = np.loadtxt(f); t = e[:, 0]; m = (t >= tr[0]) & (t <= tr[-1]); e = e[m]; t = t[m]
    P = np.column_stack([np.interp(t, tr, ref[:, k]) for k in (1, 2, 3)]); Q = e[:, 1:4]
    mp, mq = P.mean(0), Q.mean(0); U, S, Vt = np.linalg.svd((P - mp).T @ (Q - mq)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); Rm = U @ D @ Vt
    A = (Rm @ (Q - mq).T).T + mp
    ax[0].plot(A[:, 0], A[:, 1], label=f.split('/')[-2] + '/' + f.split('/')[-1].replace('.tum', '')); v = np.linalg.norm(np.gradient(Q, t, axis=0), axis=1); ax[1].plot(t - tr[0], v, label=f.split('/')[-2] + '/' + f.split('/')[-1].replace('.tum', ''))
ax[0].axis('equal'); ax[0].legend(fontsize=7); ax[1].set_ylim(0, 3); ax[1].set_xlabel('t [s]'); ax[1].set_ylabel('speed [m/s]'); ax[1].legend(fontsize=7)
plt.tight_layout(); plt.savefig(out, dpi=70)
