"""Shared plotting style for every figure of the paper.

All figures use Arial at a common set of sizes so that the typography is
consistent across the manuscript.  Import ``apply_style`` at the top of a
figure script and call it once before creating any figure::

    from plotstyle import apply_style, COL_W, save
    apply_style()

Font sizes are collected in ``SIZES`` so that a single edit rescales every
figure at once.  ``COL_W`` is the IEEE two-column text width in inches, which
is the natural width for a single-column figure.
"""
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ----------------------------------------------------------------- geometry
COL_W = 3.45          # single-column width  [in]
PAGE_W = 7.16         # full text width      [in]
# Figures are written to figures/out/ inside the repository.
FIGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(FIGDIR, exist_ok=True)

# ------------------------------------------------------------------- sizes
SIZES = {
    'title':  7.8,    # panel letters (A), (B)
    'label':  7.2,    # axis labels
    'tick':   6.4,    # tick labels
    'legend': 6.8,    # legend entries
    'annot':  6.4,    # in-plot annotations and bar value labels
}

# ------------------------------------------------------------------ colours
# Muted palette in the register used by Nature/Science and the top robotics
# journals: low saturation, no primary hues.  Every pair that has to be told
# apart is also separated in greyscale (CIE L* given), so the figures survive
# black-and-white reproduction:
#   blue 47.9 | red 49.2 | grey 58.3 | orange 63.7 | light grey 77.3
MUTED = {
    'blue':   '#4C72B0',
    'orange': '#DD8452',
    'red':    '#C44E52',
    'green':  '#55A868',
    'grey':   '#8C8C8C',
    'lgrey':  '#BFBFBF',
    'slate':  '#34495E',
}

# selection outcome, shared by Fig. 3(A) and Fig. 5
C_KEEP   = MUTED['blue']     # retained                 dL* 15.9 vs orange
C_STAGE2 = MUTED['orange']   # removed by Stage 2       dL* 13.6 vs light grey
C_STAGE1 = MUTED['lgrey']    # removed by Stage 1       dL* 29.5 vs blue

# paired bars (calibration vs validation, Rig-A vs OpenCalib)
C_CAL    = MUTED['blue']
C_VAL    = MUTED['orange']
C_CAL_E  = '#2F4F7F'
C_VAL_E  = '#A85A2E'

# distortion-order candidates                            dL* 28.1
# Two competing model variants.  The reference must be DARK: a light grey
# sits at the same tone as the dashed grid (dL* ~4) and disappears into it.
C_M2     = MUTED['slate']    # the reference      dL* 42.8 vs grid, 19.1 vs C_M3
C_M2_E   = '#22303F'         # darker edge for the reference bars
C_M3     = MUTED['red']      # the variant        dL* 23.7 vs grid
C_M3_E   = '#8C3A3D'
C_DIFF   = MUTED['blue']


def apply_style():
    """Install the common rcParams.  Call once per script."""
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['Arial', 'Liberation Sans', 'DejaVu Sans'],
        # every mathtext category must be named explicitly: the ones left at
        # their defaults fall back to Comic Sans (cursive) and cmsy10
        'mathtext.fontset': 'custom',
        'mathtext.rm': 'Arial',
        'mathtext.it': 'Arial:italic',
        'mathtext.bf': 'Arial:bold',
        'mathtext.sf': 'Arial',
        'mathtext.cal': 'Arial:italic',
        'mathtext.tt': 'Liberation Mono',
        'mathtext.default': 'it',
        'font.size': SIZES['tick'],
        'axes.titlesize': SIZES['title'],
        'axes.labelsize': SIZES['label'],
        'axes.labelpad': 1.5,
        'xtick.labelsize': SIZES['tick'],
        'ytick.labelsize': SIZES['tick'],
        'legend.fontsize': SIZES['legend'],
        'axes.linewidth': 0.6,
        'axes.grid': True,
        'axes.axisbelow': True,
        'grid.linestyle': '--',
        'grid.linewidth': 0.4,
        'grid.alpha': 0.30,
        'lines.linewidth': 1.0,
        'xtick.major.width': 0.0,   # tick marks off; labels stay
        'ytick.major.width': 0.0,
        'xtick.major.size': 0.0,
        'ytick.major.size': 0.0,
        'xtick.minor.size': 0.0,
        'ytick.minor.size': 0.0,
        'xtick.major.pad': 1.5,
        'ytick.major.pad': 1.5,
        'legend.handlelength': 1.0,
        'legend.handletextpad': 0.4,
        'legend.columnspacing': 0.8,
        'legend.borderpad': 0.1,
        'legend.labelspacing': 0.25,
        'legend.frameon': False,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.01,
        'figure.dpi': 200,
        'pdf.fonttype': 42,      # embed TrueType, keeps text selectable
        'ps.fonttype': 42,
    })


def axbox(ax, grid='y'):
    """Enclose the plotting area in a full solid frame and add a dashed grid.

    All four spines are kept, which is the convention in the robotics
    journals this paper targets.  Tick marks are suppressed; the labels stay.
    ``grid`` selects the axis the grid runs along: 'y' for categorical bar
    charts, 'both' for a continuous x-axis, or None to suppress it.
    """
    for side in ('top', 'right', 'bottom', 'left'):
        ax.spines[side].set_visible(True)
        ax.spines[side].set_linewidth(0.7)
        ax.spines[side].set_color('black')
    if grid:
        ax.set_axisbelow(True)
        ax.grid(True, axis=grid, linestyle='--', linewidth=0.4, alpha=0.30)
    else:
        ax.grid(False)
    ax.tick_params(length=0)


def xlabel(ax, text, pad=4.0):
    """x-axis title, one step larger than the y-axis title and set off from
    the axis by ``pad`` points."""
    ax.set_xlabel(text, fontsize=SIZES['label'] + 0.8, labelpad=pad)


def panel(ax, tag, x=0.035, y=0.975):
    """Bold upper-case panel letter drawn INSIDE the axes, top-left.

    Placing it inside keeps it clear of the y-axis label and keeps every
    panel letter of the paper at the same position.  The panel is *described*
    in the LaTeX caption, never inside the figure, so the letter alone is
    drawn here.
    """
    ax.text(x, y, tag, transform=ax.transAxes, ha='left', va='top',
            fontsize=SIZES['title'], fontweight='bold', zorder=6)


def save(fig, name):
    """Write ``fig`` to out/<name> and report the path."""
    path = os.path.normpath(os.path.join(FIGDIR, name))
    fig.savefig(path)
    print('wrote %s' % path)
    return path
