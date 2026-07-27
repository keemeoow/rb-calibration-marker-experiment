"""Shared publication style for every project figure.

Import and call :func:`apply_paper_style` before creating axes.  Use
:func:`save_figure` for final assets.  The visual contract follows the C1 real-data
figure set: white background, sans-serif typography, restrained colors, horizontal
grid only, open top/right spines, and high-resolution output.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt


PAPER = "#FFFFFF"
INK = "#17212B"
MUTED = "#53606B"
GRID = "#DCE2E7"
BORDER = "#B8C0C7"

# Stable semantic palettes.  Use METHOD_COLORS when categories are the C1
# calibration structures, and SERIES_COLORS for ordered raw/correction stages.
METHOD_COLORS = {
    "independent": "#5B6B7A",
    "unified_joint": "#2374AB",
    "joint_fk_fixed": "#D17A22",
}
SERIES_COLORS = {
    "raw": "#AEB8C2",
    "se3": "#6A9FB5",
    "ridge": "#1F5A94",
    "negative": "#C46A4A",
    "positive": "#2F7D67",
}
CATEGORY_COLORS = ["#5B6B7A", "#2374AB", "#D17A22", "#2F7D67", "#8064A2"]


def apply_paper_style() -> None:
    """Install the repository-wide Matplotlib defaults."""
    plt.rcParams.update({
        "font.family": ["Apple SD Gothic Neo", "Noto Sans CJK KR", "DejaVu Sans", "sans-serif"],
        "axes.unicode_minus": False,
        "figure.facecolor": PAPER,
        "axes.facecolor": PAPER,
        "savefig.facecolor": PAPER,
        "axes.edgecolor": BORDER,
        "axes.labelcolor": INK,
        "axes.titlecolor": INK,
        "text.color": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "grid.alpha": 1.0,
        "legend.frameon": False,
        "axes.titleweight": "semibold",
        "axes.titlesize": 12,
        "figure.titlesize": 16,
        "figure.titleweight": "semibold",
    })


def clean_axis(ax, *, grid_axis: str | None = "y") -> None:
    """Apply the open, low-ink axis treatment used by the reference figures."""
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(BORDER)
    ax.spines["bottom"].set_color(BORDER)
    if grid_axis:
        ax.grid(axis=grid_axis, color=GRID, linewidth=0.8, zorder=0)
        ax.set_axisbelow(True)


def save_figure(fig, path: str | Path, *, dpi: int = 220, close: bool = True) -> Path:
    """Save a publication figure with the shared background and resolution."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=dpi, bbox_inches="tight", facecolor=PAPER)
    if close:
        plt.close(fig)
    return output
