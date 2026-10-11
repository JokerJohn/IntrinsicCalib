#!/usr/bin/env python3
"""Real Kalibr (docker kalibr:noetic) at its defaults on ALL 39 Rig-A images.
  run_kalibr_all39.py <run_name>     (e.g. run1; Kalibr's view selection is random)
Model pinhole-radtan, checkerboard 11x8 inner corners, 0.045 m. The images are packed into a ROS1 bag inside the
container by make_kalibr_bag.py. The image kalibr:noetic is built from the Dockerfile_ros1_20_04 of the Kalibr
repository (override with KALIBR_IMAGE).
Output: rigd/kalibr/<run_name>/{train-camchain.yaml, kalibr.log, ...}
"""
import glob, os, subprocess, sys
RIGD = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RIGA = os.path.abspath(os.environ.get('RIG_A_DIR', os.path.join(RIGD, '..', '..', '..', 'data', 'rig_a')))
KB = os.path.dirname(os.path.abspath(__file__))   # directory of make_kalibr_bag.py
IMAGE = os.environ.get('KALIBR_IMAGE', 'kalibr:noetic')
name = sys.argv[1]
wd = os.path.join(RIGD, 'kalibr', name); os.makedirs(wd, exist_ok=True)
imgs = sorted(glob.glob(os.path.join(RIGA, '*.png'))); assert len(imgs) == 39, len(imgs)
open(os.path.join(wd, 'checkerboard.yaml'), 'w').write(
    "target_type: 'checkerboard'\ntargetCols: 11\ntargetRows: 8\nrowSpacingMeters: 0.045\ncolSpacingMeters: 0.045\n")
open(os.path.join(wd, 'train_images.txt'), 'w').write('\n'.join(os.path.basename(p) for p in imgs) + '\n')
cmd = (f"source /catkin_ws/devel/setup.bash && cd '{wd}' && rm -f train.bag train-* && "
       f"python3 '{KB}/make_kalibr_bag.py' train.bag " + ' '.join(f"'{p}'" for p in imgs) + " && "
       "rosrun kalibr kalibr_calibrate_cameras --models pinhole-radtan --target checkerboard.yaml "
       "--bag train.bag --topics /cam0/image_raw --dont-show-report; rc=$?; rm -f train.bag; exit $rc")
dcmd = ['docker', 'run', '--rm', '--network', 'none', '-u', f'{os.getuid()}:{os.getgid()}',
        '-e', 'HOME=/tmp', '-e', 'ROS_HOME=/tmp/.ros', '-e', 'MPLBACKEND=Agg',
        *[a for p in sorted({RIGD, RIGA, KB}) for a in ('-v', f'{p}:{p}')],
        '--entrypoint', '/bin/bash', IMAGE, '-c', cmd]
with open(os.path.join(wd, 'kalibr.log'), 'w') as log:
    p = subprocess.run(dcmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=3600)
print(name, 'rc', p.returncode, 'camchain exists:', os.path.exists(os.path.join(wd, 'train-camchain.yaml')))
