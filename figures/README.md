# Figure scripts

Every figure of the paper that is generated from data is produced by one script in this directory. All scripts share `plotstyle.py`, which sets the font, the font sizes and the colours, and writes every figure to `figures/out/`.

| script | output | figure |
|---|---|---|
| `make_fig2_selection.py` | `out/fig_select2.pdf` | Fig. 2, images kept and rejected per stage; four Rig-A image sets |
| `make_fig3_order.py` | `out/fig_order.pdf` | Fig. 3, distortion-order selection on Rig-A |
| `make_fig4_datasets.py` | `out/fig_datasets.pdf`, `out/rig_c_annotated.png` | Supplement Fig. 4, the four camera models (A to D), Rig-D (E, F) and Rig-C (G, H) |
| `make_perview.py` | `out/fig_perview.pdf` | per-view residuals of the 39 Rig-A images (not in the paper) |
| `make_fig6_zones.py` | `out/fig_zones.pdf` | corner-zone coverage (not in the paper) |
| `make_fig8_gui.py` | `out/gui_overview.png` | screenshot of the tool after the calibration of Rig-A (not in the paper) |
| `make_fig9_rectify.py` | `out/fig_undistort.pdf` | rectification comparison (not in the paper) |

Fig. 1 (pipeline) is drawn in TikZ in the manuscript.

## Inputs

* `data/figdata.json`: per-view errors, calibration and validation errors of both distortion orders and the radial profile of the Rig-A run (`experiments/public/experiment/rap_headless.py data/rig_a`), so that Fig. 3 and the per-view figure are redrawn without rerunning the pipeline.
* `make_fig2_selection.py` and `make_fig6_zones.py` hold the counts of Section V as literal values.
* `data/flir_full_cam{0,1}.json`: the output of `experiments/rigc/flir_full.py`, used by the Table II scripts when `flir_full.py` has not been run.
* `data/resultsB.json`: the M2 and M3 parameter sets of Rig-B used by `make_fig9_rectify.py`.
* `make_fig4_datasets.py` reads the public sessions (`experiments/public/fetch_datasets.sh`; override with `PUBLIC_DIR`), the Rig-A session `data/rig_a` (`RIG_A_DIR`), the Rig-D photos `data/rigd_platform.jpg` and `data/rigd_camera.jpg` (crops of the platform photos, box coordinates in the script refer to the uncropped photos), the Rig-C photo `data/rig_c_obscured.png` (vendor label pixelated, background blurred, cropped) and the Rig-C frame `data/rigc/frames/D2_cam0_0337.png` (session D2, cam0, frame 337; decoded from `$RIGC_DIR/calib_D2.bag` when the file is absent). Faces and readable signage are blurred before any image is written.
* `make_fig9_rectify.py` reads the outputs of `experiments/public/experiment/run_all.sh` and the Rig-A runs `out/fs39_ros.json` and `out/fs39_mrcal.json` (`experiments/public/README.md`, "Rig-A runs").
* `make_fig8_gui.py` renders the tool offscreen and requires PySide6.

## Running

```bash
python figures/make_fig2_selection.py
python figures/make_fig3_order.py
PUBLIC_DIR=experiments/public python figures/make_fig4_datasets.py
QT_QPA_PLATFORM=offscreen python figures/make_fig8_gui.py
```

`plotstyle.py` requests Arial; without the font, matplotlib falls back to DejaVu Sans with a warning, which changes the typography but not the content.
