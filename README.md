<div align="center">

<h1>Automatic Reproducible Camera Intrinsic Calibration</h1>

<p><a href="https://github.com/JokerJohn"><b>Xiangcheng Hu</b></a></p>

![IntrinsicCalib](./README/hero_gui.png)

<a href="https://arxiv.org/abs/2609.10082"><img src="https://img.shields.io/badge/arXiv-IntrinsicCalib-b31b1b" alt="arXiv"></a><a><img alt="PRs-Welcome" src="https://img.shields.io/badge/PRs-Welcome-white" /></a>[![GitHub Stars](https://img.shields.io/github/stars/JokerJohn/IntrinsicCalib.svg)](https://github.com/JokerJohn/IntrinsicCalib/stargazers)<a href="https://github.com/JokerJohn/IntrinsicCalib/network/members"><img alt="FORK" src="https://img.shields.io/github/forks/JokerJohn/IntrinsicCalib?color=white" /></a>[![GitHub Issues](https://img.shields.io/github/issues/JokerJohn/IntrinsicCalib.svg)](https://github.com/JokerJohn/IntrinsicCalib/issues)[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

</div>

## Introduction

This work targets high-accuracy camera intrinsic calibration, with a mean reprojection error below **0.2 pixel** obtained automatically, without manual image selection and without a manually specified distortion model.

Camera intrinsics are calibrated once and then held fixed throughout deployment, so a calibration error recurs in every downstream task as a systematic geometric bias. The attainable accuracy depends on two decisions that existing toolboxes leave to the operator: which images to estimate from, and which radial distortion order to adopt. Both are determined here from the collected data:

- **Image selection.** Views whose mean residual exceeds a multiple of the median are rejected iteratively. The rejection is executed independently under each candidate distortion order, so the retained image set is consistent with the residual scale of that order.
- **Distortion-order selection.** Each candidate is scored on held-out images, with the intrinsics and distortion fixed and only the board pose re-estimated, so an added coefficient is adopted only when it is supported by independent observations.
- **Interactive tool.** Both steps are integrated into a calibration tool that supports full-pipeline data inspection and parameter estimation.

Experiments on our own camera data and five public datasets show that image filtering reduces the held-out reprojection error by 25% and order selection by a further 5%, achieving the lowest held-out mean among four compared configurations. On our own rig the pipeline attains 0.143 px on the retained images, against 0.151 px for mrcal and 0.299 px for the ROS calibrator, and the held-out error reaches 0.148 px on the public datasets.

<div align="center">

![Pipeline](./README/pipeline.png)

</div>

## News

- **2026/09/09**: [arXiv preprint](https://arxiv.org/abs/2609.10082) online; code, tool and data are being prepared for release.

## Cameras and Datasets

<div align="center">

![Datasets](./README/datasets.png)

</div>

<div align="center">

| Dataset | Camera | Images | Board |
| ------- | ------ | -----: | ----- |
| `Rig-A` | 1920×1080, 111° | 39 | 11×8, 45 mm |
| `Rig-B` | same rig, 2 sessions | — | 11×8, 45 mm |
| `OpenCalib-F` | 1920×1200, 84° | 22 | 17×15, 50 mm |
| `ROS L/R` | 640×480, 73° | 12 | 8×6, 108 mm |
| `OpenCV L/R` | 640×480, 62° | 13 | 9×6, 30 mm |

</div>

`Rig-A` and `Rig-B` are captured hand-held on our own mobile mapping rig; the remaining datasets are those distributed with [OpenCalib](https://github.com/PJLab-ADG/SensorsCalibration), ROS and OpenCV. Download links will be added on release.

## Interactive Calibration Tool

<div align="center">

![GUI](./README/gui.png)

</div>

The workflow consists of three steps: importing an image folder, specifying the target geometry, and running the calibration. The tool displays the filtering status of every image and the distribution of calibration points, so that both data-dependent decisions can be inspected within the interface. Alongside the estimated parameters it records the calibration provenance: the images retained and rejected at each stage with their residuals, the estimation and validation subsets, the four quantities behind the order decision, and the number of observations in the four 20% corner zones of the image. The algorithmic core is independent of the interface and can be invoked from scripts; all reported results are produced in this way.

<div align="center">

![Architecture](./README/arch.png)

</div>

## Method

Calibration over an image set $\mathcal{S}$ estimates the intrinsics $K$, the distortion $d$ and one board pose $T_i$ per image by minimising the reprojection error:

<div align="center">

$$
\min_{K,\,d,\,\{T_i\}} \sum_{i \in \mathcal{S}} \sum_{j=1}^{N_i} \left\| u_{ij} - \pi(K, d, T_i, P_j) \right\|_2^2
$$

</div>

**Image selection.** After an initial fit, each view is summarised by its mean residual. Views exceeding twice the median of the retained set are rejected, after which the fit is re-solved. The median is recomputed at every iteration, so the threshold tightens as outlier views are removed:

<div align="center">

$$
\tau^{(t)} = \kappa\,\tilde{e}^{(t)}, \quad \kappa = 2, \qquad
\mathcal{I}^{(t+1)} = \left\lbrace\, i \in \mathcal{I}^{(t)} \;\middle|\; \bar{e}_i^{(t)} \le \tau^{(t)} \,\right\rbrace
$$

</div>

> This threshold depends on the distortion order: a higher order fits the same corners more closely and therefore yields a lower residual scale, so an image set fixed in advance is consistent with neither candidate. The rejection is accordingly executed independently within each candidate order.

**Distortion-order selection.** Every fifth image in capture order is assigned to a validation subset. On these views the intrinsics and distortion are held fixed and only the six-DoF board pose is re-estimated, so each candidate is evaluated on views that did not contribute to its own estimation:

<div align="center">

$$
\hat{T}_i^{\mathcal{M}} = \arg\min_{T \in \mathrm{SE}(3)} \sum_{j=1}^{N_i} \left\| u_{ij} - \pi(K_{\mathcal{M}}, d_{\mathcal{M}}, T, P_j) \right\|_2^2
$$

</div>

The additional radial coefficient $k_3$ is adopted only when it degrades neither the mean nor the worst-case validation error:

<div align="center">

$$
\bar{e}_{\mathrm{v}}(\mathcal{M}_3) \le \alpha\,\bar{e}_{\mathrm{v}}(\mathcal{M}_2)
\quad\text{and}\quad
e^{\max}_{\mathrm{v}}(\mathcal{M}_3) \le \beta\,e^{\max}_{\mathrm{v}}(\mathcal{M}_2)
$$

</div>

## Results

| ![Selection](./README/selection.png) | ![Order](./README/order.png) |
| ------------------------------------ | ---------------------------- |
| ![Per-view](./README/perview.png) | ![Comparison](./README/table.png) |

## Getting Started

> The calibration tool has not been released yet; the commands below describe the intended workflow.

The implementation is in Python and targets Ubuntu 20.04/22.04 with OpenCV and PySide6.

```bash
git clone https://github.com/JokerJohn/IntrinsicCalib.git
cd IntrinsicCalib
scripts/setup_env.sh          # conda environment
scripts/run_gui.sh            # launch the tool
```

Select a folder of chessboard images, specify the board geometry, then run `Detect All` and `Calibrate`. Each selection stage can be disabled individually, which reproduces the ablations reported above.

## TODO

- [ ] Release the calibration tool and the evaluation scripts
- [ ] Release the Rig-A and Rig-B image sets
- [ ] Add rational and fisheye models to the candidate set
- [ ] Incorporate corner-coverage constraints into the filtering criterion

## Citation

```bibtex
@article{hu2026intrinsiccalib,
  title   = {Automatic Reproducible Camera Intrinsic Calibration},
  author  = {Hu, Xiangcheng},
  journal = {arXiv preprint arXiv:2609.10082},
  eprint  = {2609.10082},
  year    = {2026}
}
```

## License

Released under the [MIT license](./LICENSE).

## Acknowledgment

We thank the maintainers of [OpenCalib](https://github.com/PJLab-ADG/SensorsCalibration), [ROS camera_calibration](http://wiki.ros.org/camera_calibration), [OpenCV](https://github.com/opencv/opencv) and [mrcal](https://mrcal.secretsauce.net) for the public datasets and the baseline implementations used in this evaluation.

## Contributors

<a href="https://github.com/JokerJohn/IntrinsicCalib/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=JokerJohn/IntrinsicCalib" />
</a>
