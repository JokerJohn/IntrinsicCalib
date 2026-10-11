"""Reference trajectories of the VIO evaluation, in one place.

  ref set 'gt'  : ground-truth trajectories (Rig-C: FAST-LIO2 + manual loop closures; Rig-D: prior-map
                  localisation + manual loop closures; gt/README_*.md)

Sequence keys: the three Rig-C bag stamps and 'rigd_office1'.
The gt files are searched in $VIO_GT_DIR, then gt/ next to this file.
"""
import os
HERE = os.path.dirname(os.path.abspath(__file__))
GT_DIRS = [d for d in (os.environ.get('VIO_GT_DIR'), f'{HERE}/gt') if d]
REFS = {
    '2026-02-23-11-46-32': {'gt': 'gt_rigc_s1.txt'},       # Rig-C S1
    '2026-02-23-11-57-46': {'gt': 'gt_rigc_s2.txt'},       # Rig-C S2
    '2026-02-22-11-25-57': {'gt': 'gt_rigc_s3.txt'},       # Rig-C S3
    'rigd_office1':         {'gt': 'gt_rigd_office1.txt'},  # Rig-D
}
REF_SETS = ('gt',)
def ref_path(ref_set, key):
    """Absolute path of the reference trajectory of sequence `key` in `ref_set`."""
    ent = REFS[key]
    if ref_set == 'gt': cands = [f'{d}/{ent["gt"]}' for d in GT_DIRS]
    else: raise ValueError(f'unknown reference set {ref_set!r}; choose from {REF_SETS}')
    for c in cands:
        if os.path.exists(c): return os.path.realpath(c)
    raise FileNotFoundError(f'{ref_set} reference of {key} not found; looked in: ' + ', '.join(cands))
def add_args(ap):
    ap.add_argument('--ref-set', choices=REF_SETS, default='gt', help='reference trajectories: gt = ground truth')
    ap.add_argument('--rescore-only', action='store_true', help='re-score the existing trajectories only, no VIO run')
