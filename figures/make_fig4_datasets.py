#!/usr/bin/env python
"""Supplement Fig. 4: one frame per evaluated camera model, Rig-D and Rig-C.

Rows 1-2 (A-D) are central 16:9 crops annotated with the resolution and the
diagonal field of view.  Row 3 is the Rig-D wheeled robot (E) and its camera
close-up (F); both photos are those of the P2Calib platform figure, recropped
to 4:3 and annotated differently, and the sensor box on E is linked to F by
zoom guides.  Row 4 is the handheld Rig-C sensor head (G) and one frame of its
AprilGrid session (H).  Faces are blurred before anything is
written to disk: the Haar cascades catch most of them and MANUAL_BLUR lists
the boxes they miss on these specific frames.  On the Rig-C photo the vendor
sticker is pixelated and blurred, and the table behind the device is blurred
through a feathered device mask.
"""
import os, sys
import numpy as np
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plotstyle import apply_style, save, SIZES, COL_W, FIGDIR
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
# public sessions (experiments/public/fetch_datasets.sh) and the Rig-A session
PUB = os.environ.get('PUBLIC_DIR', os.path.join(ROOT, 'experiments', 'public'))
FS = os.environ.get('RIG_A_DIR', os.path.join(ROOT, 'data', 'rig_a'))

# Public sessions first, our own rig last (panel D).
PANELS = [
    ('OpenCV',        '640 \u00d7 480, 62\u00b0',    PUB + '/opencv_left/left03.jpg'),
    ('OpenCalib',   '1920 \u00d7 1200, 84\u00b0',  PUB + '/opencalib_front/4.png'),
    ('ROS',           '640 \u00d7 480, 73\u00b0',    PUB + '/ros_wide_left/left0008.png'),
    ('Rig-A / Rig-B', '1920 \u00d7 1080, 111\u00b0', FS + '/left-0030.png'),
]

def _haarcascade_dir():
    """Directory of the OpenCV Haar cascades: cv2.data when the wheel ships it,
    otherwise <sys.prefix>/share/opencv4/haarcascades, or $OPENCV_HAARCASCADES."""
    d = os.environ.get('OPENCV_HAARCASCADES')
    if d:
        return d.rstrip('/') + '/'
    if hasattr(cv2, 'data'):
        return cv2.data.haarcascades
    return os.path.join(sys.prefix, 'share', 'opencv4', 'haarcascades') + '/'


CASC = _haarcascade_dir()
det_face = cv2.CascadeClassifier(CASC + 'haarcascade_frontalface_default.xml')
det_prof = cv2.CascadeClassifier(CASC + 'haarcascade_profileface.xml')


# Faces the Haar cascades miss on these specific frames, in source pixels.
MANUAL_BLUR = {
    'ros_wide_left/left0008.png': [(290, 104, 48, 50)],
    'left-0030.png': [(1148, 388, 64, 62)],
}


def blur_box(img, x, y, w, h):
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(img.shape[1], x + w), min(img.shape[0], y + h)
    roi = img[y0:y1, x0:x1]
    if roi.size:
        k = max(9, (min(roi.shape[:2]) // 2) * 2 + 1)
        img[y0:y1, x0:x1] = cv2.GaussianBlur(roi, (k, k), 0)


def anonymize(img, max_frac=0.12, exclude=None):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    boxes = []
    for det in (det_face, det_prof):
        for sf, mn in ((1.05, 4), (1.10, 3)):
            boxes += list(det.detectMultiScale(g, sf, mn, minSize=(20, 20)))
    H, W = img.shape[:2]
    n = 0
    for (x, y, w, h) in boxes:
        if max(w, h) > max_frac * W:
            continue
        if exclude is not None:        # (x0, y0, x1, y1): calibration target
            ex0, ey0, ex1, ey1 = exclude
            if x < ex1 and x + w > ex0 and y < ey1 and y + h > ey0:
                continue
        pad = int(0.35 * max(w, h))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(W, x + w + pad), min(H, y + h + pad)
        roi = img[y0:y1, x0:x1]
        if roi.size:
            k = max(9, (min(roi.shape[:2]) // 2) * 2 + 1)
            img[y0:y1, x0:x1] = cv2.GaussianBlur(roi, (k, k), 0)
            n += 1
    return img, n


# ------------------------------------------------------------------ Rig-C
# The photo of the sensor head is distributed only in its obscured form (vendor
# sticker pixelated, background blurred, already cropped): data/rig_c_obscured.png.
# RIG_C_PHOTO may point to an unprocessed photo, which is then processed below.
RIG_OBSCURED = os.path.join(HERE, 'data', 'rig_c_obscured.png')
RIG_PHOTO = os.environ.get('RIG_C_PHOTO', os.path.join(FIGDIR, 'rig_c.jpg'))
# Frame RIG_FRAME of session D2, cam0, shipped in data/rigc/frames/; otherwise
# decoded from the D2 bag in $RIGC_DIR (data/rigc/MANIFEST.md).
RIG_FRAME_PNG = os.path.join(ROOT, 'data', 'rigc', 'frames', 'D2_cam0_0337.png')
RIGC_DIR = os.environ.get('RIGC_DIR', os.path.join(ROOT, 'data', 'rigc'))
RIG_FRAME = 337                      # session D2, cam0: 9/9 tags, 36/36 corners
RIG_SPEC = ('Rig-C', '1440 × 1080, 113°')
# Wheeled platform photos (2880 x 2160 and 2160 x 2880), shared with the P2Calib
# platform figure.  data/rigd_platform.jpg and data/rigd_camera.jpg hold the
# PLAT_CROP and CLOSE_CROP regions of the original photos; the box coordinates
# below are in original photo pixels.
PLATFORM = os.path.join(HERE, 'data', 'rigd_platform.jpg')
PLAT_CROP = (180, 990, 1540, 2010)    # 4:3, vehicle only, target board excluded
PLAT_MOUNT = (712, 1035, 1128, 1400)  # sensor enclosure, photo pixels
CLOSEUP = os.path.join(HERE, 'data', 'rigd_camera.jpg')
CLOSE_CROP = (400, 800, 1760, 1820)   # 4:3 around the sensor module (2160 x 2880)
CLOSE_CAM = (610, 1160, 905, 1460)    # camera lens, photo pixels
CLOSE_LID = (940, 1145, 1550, 1480)   # LiDAR used for the ground truth, photo pixels
ZOOM_RED = '#D62728'


# Sensor-head photo (1512 x 1536), source pixels.  The device mask is the union
# of the LiDAR outline, the enclosure and the two lens barrels.
LIDAR_POLY = [(540, 198), (958, 198), (962, 445), (992, 500), (994, 596),
              (502, 596), (502, 500), (533, 445)]
ENCL_RECT = (448, 584, 1054, 1250)
CAM_ELL = [((270, 866), (186, 172)), ((1232, 872), (184, 172))]   # centre, axes
BARREL = [(300, 702, 460, 1012), (1040, 702, 1110, 1012)]
STICKER = (462, 622, 852, 828)       # vendor sticker incl. 12-16 px margin
RIG_CROP = (16, 168, 1482, 1268)     # 4:3, whole device with a table margin

# annotation boxes (photo pixels) in the style of the P2Calib platform figure
ANN_BLUE = '#83D4F6'
ANN_LIDAR = (500, 192, 998, 600)
ANN_CAM1 = (76, 686, 462, 1048)      # image left  = cam1 = right camera (see note)
ANN_CAM0 = (1040, 690, 1424, 1052)   # image right = cam0 = left camera
# Kalibr's T_cn_cnm1 maps cam0 coordinates to cam1 coordinates; its translation
# (-0.1176, 0, 0) is the centre of cam0 seen from cam1, i.e. cam0 lies on the
# rig's left (camera x points right).  T_cam_imu agrees: the camera x axis is
# IMU -x, and cam0 sits at IMU x = +0.037 m, cam1 at -0.081 m.  The photo looks
# at the lenses from the front, so the rig's left appears on the image right.

RIG_GRID_BOX = (540, 385, 975, 815)    # printed AprilGrid sheet in the frame
# Rig-C frame: readable signage to blur (x, y, w, h), source pixels
RIG_TEXT_BLUR = [(684, 300, 120, 88),    # hanging banner above the grid
                 (148, 404, 56, 100),    # wall poster, left
                 (40, 455, 80, 70),      # wall poster, far left
                 (320, 880, 80, 90)]     # carton print, bottom left


def device_mask(shape):
    m = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(m, [np.array(LIDAR_POLY, np.int32)], 255)
    x0, y0, x1, y1 = ENCL_RECT
    cv2.rectangle(m, (x0, y0), (x1, y1), 255, -1)
    for (c, ax) in CAM_ELL:
        cv2.ellipse(m, c, ax, 0, 0, 360, 255, -1)
    for (x0, y0, x1, y1) in BARREL:
        cv2.rectangle(m, (x0, y0), (x1, y1), 255, -1)
    m = cv2.GaussianBlur(m, (0, 0), 6)           # feathered edge, ~15 px
    return m.astype(np.float32)[..., None] / 255.0


def process_rig_photo():
    if not os.environ.get('RIG_C_PHOTO') and os.path.exists(RIG_OBSCURED):
        return cv2.cvtColor(cv2.imread(RIG_OBSCURED), cv2.COLOR_BGR2RGB)
    im = cv2.imread(RIG_PHOTO)
    assert im is not None, RIG_PHOTO
    # 1) obliterate the vendor sticker: 28 px pixelation, then a heavy blur
    x0, y0, x1, y1 = STICKER
    roi = im[y0:y1, x0:x1]
    h, w = roi.shape[:2]
    small = cv2.resize(roi, (max(1, w // 28), max(1, h // 28)),
                       interpolation=cv2.INTER_AREA)
    roi = cv2.resize(small, (w, h), interpolation=cv2.INTER_NEAREST)
    im[y0:y1, x0:x1] = cv2.GaussianBlur(roi, (0, 0), 14)
    # 2) blur everything outside the feathered device mask
    m = device_mask(im.shape)
    bg = cv2.GaussianBlur(im, (0, 0), 22)
    im = (m * im + (1.0 - m) * bg).round().astype(np.uint8)
    x0, y0, x1, y1 = RIG_CROP
    return cv2.cvtColor(im[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)


RIG_BAG = os.path.join(RIGC_DIR, 'calib_D2.bag')


def read_bag_frame(bag, topic, index):
    """Grayscale frame `index` of a CompressedImage topic (rosbags reader)."""
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    with AnyReader([Path(bag)]) as rd:
        con = [c for c in rd.connections if c.topic == topic]
        for k, (c, _t, raw) in enumerate(rd.messages(connections=con)):
            if k == index:
                m = rd.deserialize(raw, c.msgtype)
                return cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8),
                                    cv2.IMREAD_GRAYSCALE)
    raise IndexError(index)


def rig_frame():
    if os.path.exists(RIG_FRAME_PNG):
        f = cv2.imread(RIG_FRAME_PNG, cv2.IMREAD_GRAYSCALE)
    else:
        f = read_bag_frame(RIG_BAG, '/sensors/cam0/compressed', RIG_FRAME)
    im = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)
    # The AprilGrid tags trigger the frontal-face cascade; no person is in
    # view, so detections on the target or larger than a far face are skipped.
    im, k = anonymize(im, max_frac=0.06, exclude=RIG_GRID_BOX)
    for bx in RIG_TEXT_BLUR:
        blur_box(im, *bx)
    return cv2.cvtColor(im, cv2.COLOR_BGR2RGB), k


def annotate_rig(a):
    """Sensor boxes and arrowed labels; coordinates in photo pixels."""
    ox, oy = RIG_CROP[0], RIG_CROP[1]

    def box(r, lw=1.1):
        x0, y0, x1, y1 = r[0] - ox, r[1] - oy, r[2] - ox, r[3] - oy
        a.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                              edgecolor=ANN_BLUE, lw=lw, zorder=5,
                              path_effects=[pe.Stroke(linewidth=lw + 0.9,
                                                      foreground='#17212A'),
                                            pe.Normal()]))
        return x0, y0, x1, y1

    def label(text, xy, xytext):
        a.annotate(text, xy=xy, xytext=xytext, fontsize=SIZES['annot'],
                   color='white', ha='center', va='center', zorder=7,
                   bbox=dict(boxstyle='round,pad=0.25', facecolor='#34495E',
                             edgecolor=ANN_BLUE, linewidth=0.7),
                   arrowprops=dict(arrowstyle='-|>', color=ANN_BLUE, lw=1.0,
                                   mutation_scale=6, shrinkA=0, shrinkB=1.5))

    lx0, ly0, lx1, ly1 = box(ANN_LIDAR)
    c0 = box(ANN_CAM0)
    c1 = box(ANN_CAM1)
    label('LiDAR', (lx0, (ly0 + ly1) * 0.5), (190, 360))
    label('cam R', ((c1[0] + c1[2]) / 2, c1[3]), (250, 1010))
    label('cam L', ((c0[0] + c0[2]) / 2, c0[3]), (1215, 1010))
    # the IMU is integrated in the Ouster LiDAR
    label('IMU', (lx1, (ly0 + ly1) * 0.5), (1290, 360))


apply_style()
plt.rcParams.update({'axes.grid': False})

imgs, nfaces = [], 0
for _, _, p in PANELS:
    im = cv2.imread(p)
    assert im is not None, p
    im, k = anonymize(im)
    nfaces += k
    for key, boxes in MANUAL_BLUR.items():
        if p.endswith(key):
            for bx in boxes:
                blur_box(im, *bx)
                nfaces += 1
    h, w = im.shape[:2]                      # central 16:9 crop, equal height
    hh = int(round(w * 9.0 / 16.0))
    if hh < h:
        y0 = (h - hh) // 2
        im = im[y0:y0 + hh]
    imgs.append(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))

plat = cv2.imread(PLATFORM)
assert plat is not None, PLATFORM
plat = cv2.cvtColor(plat, cv2.COLOR_BGR2RGB)          # already the PLAT_CROP region
close = cv2.imread(CLOSEUP)
assert close is not None, CLOSEUP
close = cv2.cvtColor(close, cv2.COLOR_BGR2RGB)        # already the CLOSE_CROP region
rig = process_rig_photo()
frm, k = rig_frame()
nfaces += k
imgs += [plat, close, rig, frm]

TITLES = ['%s  (%s)' % (n, s) for n, s, _ in PANELS] + \
         ['Rig-D wheeled robot', 'Rig-D camera', 'Rig-C sensor head',
          '%s  (%s)' % RIG_SPEC]

ars = [im.shape[1] / im.shape[0] for im in imgs]
# single-column grid: two rows of two 16:9 panels, one row of three 4:3 panels
W, COL_GAP, ROW_GAP, TITLE = COL_W, 0.030, 0.030, 0.140   # inches
ROWS = [[0, 1], [2, 3], [4, 5], [6, 7]]
NR = len(ROWS)
row_h = [(W - COL_GAP * (len(R) - 1)) / sum(ars[k] for k in R) for R in ROWS]
H = sum(row_h) + NR * TITLE + (NR - 1) * ROW_GAP
fig = plt.figure(figsize=(W, H))
AX = {}

y = H
for r, R in enumerate(ROWS):
    y -= TITLE
    x = 0.0
    for k in R:
        h = row_h[r]
        cw = h * ars[k]
        a = fig.add_axes([x / W, (y - h) / H, cw / W, h / H])
        x += cw + COL_GAP
        AX[k] = a
        a.imshow(imgs[k], aspect='auto')
        a.set_xticks([]); a.set_yticks([])
        for sp in a.spines.values():
            sp.set_linewidth(0.4); sp.set_color('#999999')
        # dataset name first, sensor specs subordinated in parentheses
        a.set_title(TITLES[k], fontsize=SIZES['title'], pad=3.0)
        if k == 6:
            annotate_rig(a)
        # panel letter in the top-right corner, so the caption can refer to it
        a.text(0.975, 0.94, 'ABCDEFGH'[k], transform=a.transAxes,
               ha='right', va='top', fontsize=SIZES['title'],
               fontweight='bold', color='black',
               bbox=dict(boxstyle='square,pad=0.18', facecolor='white',
                         edgecolor='none', alpha=0.80))
    y -= row_h[r] + ROW_GAP

# zoom-inset convention: red box on the sensor mount, F framed in the same red,
# dashed guides from the box's right-hand corners to F's left-hand corners
from matplotlib.patches import ConnectionPatch
aE, aF = AX[4], AX[5]
mx0, my0 = PLAT_MOUNT[0] - PLAT_CROP[0], PLAT_MOUNT[1] - PLAT_CROP[1]
mx1, my1 = PLAT_MOUNT[2] - PLAT_CROP[0], PLAT_MOUNT[3] - PLAT_CROP[1]
aE.add_patch(Rectangle((mx0, my0), mx1 - mx0, my1 - my0, fill=False,
                       edgecolor=ZOOM_RED, lw=1.0, zorder=5,
                       path_effects=[pe.Stroke(linewidth=1.9, foreground='#17212A'),
                                     pe.Normal()]))
# camera lens in the close-up, in the style of the Rig-C labels
cx0, cy0 = CLOSE_CAM[0] - CLOSE_CROP[0], CLOSE_CAM[1] - CLOSE_CROP[1]
cx1, cy1 = CLOSE_CAM[2] - CLOSE_CROP[0], CLOSE_CAM[3] - CLOSE_CROP[1]
aF.add_patch(Rectangle((cx0, cy0), cx1 - cx0, cy1 - cy0, fill=False,
                       edgecolor=ANN_BLUE, lw=1.1, zorder=5,
                       path_effects=[pe.Stroke(linewidth=2.0, foreground='#17212A'),
                                     pe.Normal()]))
aF.annotate('camera', xy=((cx0 + cx1) / 2, cy0), xytext=((cx0 + cx1) / 2, 150),
            fontsize=SIZES['annot'], color='white', ha='center', va='center',
            zorder=7, bbox=dict(boxstyle='round,pad=0.25', facecolor='#34495E',
                                edgecolor=ANN_BLUE, linewidth=0.7),
            arrowprops=dict(arrowstyle='-|>', color=ANN_BLUE, lw=1.0,
                            mutation_scale=6, shrinkA=0, shrinkB=1.5))
lx0, ly0 = CLOSE_LID[0] - CLOSE_CROP[0], CLOSE_LID[1] - CLOSE_CROP[1]
lx1, ly1 = CLOSE_LID[2] - CLOSE_CROP[0], CLOSE_LID[3] - CLOSE_CROP[1]
aF.add_patch(Rectangle((lx0, ly0), lx1 - lx0, ly1 - ly0, fill=False,
                       edgecolor=ANN_BLUE, lw=1.1, zorder=5,
                       path_effects=[pe.Stroke(linewidth=2.0, foreground='#17212A'),
                                     pe.Normal()]))
aF.annotate('LiDAR', xy=((lx0 + lx1) / 2, ly0), xytext=((lx0 + lx1) / 2, 150),
            fontsize=SIZES['annot'], color='white', ha='center', va='center',
            zorder=7, bbox=dict(boxstyle='round,pad=0.25', facecolor='#34495E',
                                edgecolor=ANN_BLUE, linewidth=0.7),
            arrowprops=dict(arrowstyle='-|>', color=ANN_BLUE, lw=1.0,
                            mutation_scale=6, shrinkA=0, shrinkB=1.5))
for sp in aF.spines.values():
    sp.set_edgecolor(ZOOM_RED); sp.set_linewidth(1.0)
hF = close.shape[0]
for yf, yt in ((my0, 0), (my1, hF)):
    fig.add_artist(ConnectionPatch(xyA=(mx1, yf), coordsA=aE.transData,
                                   xyB=(0, yt), coordsB=aF.transData,
                                   color=ZOOM_RED, lw=0.6,
                                   linestyle=(0, (2.2, 1.6)), zorder=4))

print('blurred %d face boxes' % nfaces)
save(fig, 'fig_datasets.pdf')

# stand-alone annotated Rig-C photo for reuse, same print scale as panel E
cell_w = (W - COL_GAP) / 2.0
hE = cell_w / ars[6]
fe = plt.figure(figsize=(cell_w, hE))
a = fe.add_axes([0, 0, 1, 1])
a.imshow(rig, aspect='auto')
a.set_xticks([]); a.set_yticks([])
for sp in a.spines.values():
    sp.set_visible(False)
annotate_rig(a)
fe.savefig(os.path.join(FIGDIR, 'rig_c_annotated.png'), dpi=900,
           bbox_inches=None, pad_inches=0)
print('wrote %s' % os.path.join(FIGDIR, 'rig_c_annotated.png'))
